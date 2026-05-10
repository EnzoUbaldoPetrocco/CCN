import pandas as pd
import numpy as np
from scipy.stats import chi2
from sklearn.ensemble import IsolationForest
from datetime import datetime
import os

def calculate_mahalanobis(df):
    """
    Calculates Mahalanobis Distance for a set of observations.
    Accounts for the correlation between aggregated dimensions.
    """
    df_numeric = df.select_dtypes(include=[np.number])
    mean = df_numeric.mean()
    
    try:
        # Compute the covariance matrix and its inverse
        cov = np.cov(df_numeric.values.T)
        inv_cov = np.linalg.inv(cov)
    except np.linalg.LinAlgError:
        # Error handling for singular matrices (highly correlated features)
        print("Warning: Covariance matrix is singular. Check for redundant variables.")
        return pd.Series([0] * len(df), index=df.index)
        
    diff = df_numeric - mean
    md = []
    for i in range(len(diff)):
        # Formula: sqrt((x - mu)^T * S^-1 * (x - mu))
        md.append(np.sqrt(diff.iloc[i].values.T @ inv_cov @ diff.iloc[i].values))
    
    return pd.Series(md, index=df.index)

def main():
    # --- Configuration ---
    input_file = "../intro/silver_layer/intro_study_silver.csv"
    id_col = 'Participant_ID'
    
    # Load dataset
    if not os.path.exists(input_file):
        print(f"File not found: {input_file}")
        return
    df = pd.read_csv(input_file)

    # --- 1. Aggregation Phase ---
    # Constructing stable composite scores
    df_agg = pd.DataFrame()
    df_agg[id_col] = df[id_col]

    # Culture Affinity (Mean Q1-Q3)
    df_agg['Culture_Score'] = df[[f'Culture_Affinity_Q{i}' for i in range(1, 4)]].mean(axis=1)

    # Robot Trust (Mean Q1-Q14)
    df_agg['Trust_Score'] = df[[f'Robot_Trust_Q{i}' for i in range(1, 15)]].mean(axis=1)

    # BFI-10 Personality Traits (OCEAN)
    # Reverse scoring for BFI-10 (assuming Likert 1-5 scale)
    def rev(x): return 6 - x

    df_agg['Extraversion'] = (df['Personality_BFI_Q1'].apply(rev) + df['Personality_BFI_Q6']) / 2
    df_agg['Agreeableness'] = (df['Personality_BFI_Q2'] + df['Personality_BFI_Q7'].apply(rev)) / 2
    df_agg['Conscientiousness'] = (df['Personality_BFI_Q3'].apply(rev) + df['Personality_BFI_Q8']) / 2
    df_agg['Neuroticism'] = (df['Personality_BFI_Q4'].apply(rev) + df['Personality_BFI_Q9']) / 2
    df_agg['Openness'] = (df['Personality_BFI_Q5'].apply(rev) + df['Personality_BFI_Q10']) / 2

    # --- 2. Multivariate Detection Phase ---
    analysis_features = df_agg.drop(columns=[id_col])
    
    # A. Mahalanobis Distance Logic
    df_agg['Mahalanobis_Dist'] = calculate_mahalanobis(analysis_features)
    # P-value based on Chi-Square distribution (df = number of features)
    df_agg['P_Value_MD'] = 1 - chi2.cdf(df_agg['Mahalanobis_Dist']**2, df=len(analysis_features.columns))
    mahalanobis_outliers = df_agg[df_agg['P_Value_MD'] < 0.001]

    # B. Isolation Forest Logic
    # contamination=0.05 (identifies top 5% as outliers)
    iso = IsolationForest(contamination=0.05, random_state=42)
    df_agg['IsoForest_Label'] = iso.fit_predict(analysis_features)
    iso_outliers = df_agg[df_agg['IsoForest_Label'] == -1]

    # --- 3. Reporting & Persistence ---
    # Merge findings to identify participants flagged by multiple methods
    all_outlier_ids = list(set(mahalanobis_outliers[id_col]) | set(iso_outliers[id_col]))
    
    report_data = []
    for p_id in all_outlier_ids:
        row = df_agg[df_agg[id_col] == p_id].iloc[0]
        report_data.append({
            id_col: p_id,
            'Is_MD_Outlier': p_id in mahalanobis_outliers[id_col].values,
            'Is_IsoForest_Outlier': p_id in iso_outliers[id_col].values,
            'MD_Score': round(row['Mahalanobis_Dist'], 2),
            'P_Value': round(row['P_Value_MD'], 4)
        })

    if report_data:
        report_df = pd.DataFrame(report_data)
        timestamp = datetime.now().strftime("%Y%m%d_%H%M")
        output_file = f"multivariate_outliers_{timestamp}.csv"
        report_df.to_csv(output_file, index=False)
        
        print(f"\nMultivariate Analysis Complete. Found {len(report_data)} unique candidates.")
        print(f"Results saved to: {output_file}")
        print("\nBreakdown:")
        print(f"- Mahalanobis Outliers: {len(mahalanobis_outliers)}")
        print(f"- Isolation Forest Outliers: {len(iso_outliers)}")
    else:
        print("\nNo multivariate outliers detected.")

if __name__ == "__main__":
    main()