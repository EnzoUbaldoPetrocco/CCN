import pandas as pd
import numpy as np
from datetime import datetime

def get_outlier_participant_ids(df, id_col='Participant_ID', threshold=2.5):
    """
    Identifies Participant IDs associated with outliers.
    """
    outlier_report = {}
    
    numeric_cols = df.select_dtypes(include=[np.number]).columns
    cols_to_analyze = [col for col in numeric_cols if col != id_col]
    
    for col in cols_to_analyze:
        series = df[col]
        median = series.median()
        mad = (series - median).abs().median()
        
        if mad == 0:
            continue
            
        mod_z = 0.6745 * (series - median) / mad
        is_outlier = mod_z.abs() > threshold
        outlier_ids = df.loc[is_outlier, id_col].unique().tolist()
        
        if outlier_ids:
            outlier_report[col] = outlier_ids
            
    return outlier_report

def main():
    # --- Configuration ---
    input_file = "../intro/silver_layer/intro_study_silver.csv"
    id_col = 'Participant_ID'
    threshold = 3.0
    
    try:
        df = pd.read_csv(input_file)
    except FileNotFoundError:
        print(f"File not found: {input_file}")
        return

    # --- 1. Data Transformation & Aggregation ---
    df_agg = pd.DataFrame()
    df_agg[id_col] = df[id_col]

    # A. Culture Affinity (Mean of Q1-Q3)
    affinity_cols = [f'Culture_Affinity_Q{i}' for i in range(1, 4)]
    df_agg['Culture_Affinity_Score'] = df[affinity_cols].mean(axis=1)

    # B. Robot Trust (Mean of Q1-Q14)
    trust_cols = [f'Robot_Trust_Q{i}' for i in range(1, 15)]
    df_agg['Robot_Trust_Score'] = df[trust_cols].mean(axis=1)

    # C. Personality (BFI-10 / OCEAN Extraction)
    # Mapping based on Rammstedt & John (2007) for BFI-10
    # Reverse scoring: (Max + Min) - Score. Assuming Likert 1-5.
    def reverse(x): return 6 - x

    # Extraversion: Q1(R), Q6
    df_agg['Extraversion'] = (df['Personality_BFI_Q1'].apply(reverse) + df['Personality_BFI_Q6']) / 2
    # Agreeableness: Q2, Q7(R)
    df_agg['Agreeableness'] = (df['Personality_BFI_Q2'] + df['Personality_BFI_Q7'].apply(reverse)) / 2
    # Conscientiousness: Q3(R), Q8
    df_agg['Conscientiousness'] = (df['Personality_BFI_Q3'].apply(reverse) + df['Personality_BFI_Q8']) / 2
    # Neuroticism: Q4(R), Q9
    df_agg['Neuroticism'] = (df['Personality_BFI_Q4'].apply(reverse) + df['Personality_BFI_Q9']) / 2
    # Openness: Q5(R), Q10
    df_agg['Openness'] = (df['Personality_BFI_Q5'].apply(reverse) + df['Personality_BFI_Q10']) / 2

    print(f"Aggregated columns created: {list(df_agg.columns[1:])}")

    # --- 2. Outlier Detection on Aggregated Data ---
    outliers = get_outlier_participant_ids(df_agg, id_col=id_col, threshold=threshold)
    
    if outliers:
        print("\nOutlier Participant IDs by Aggregated Dimension:")
        for col, ids in outliers.items():
            print(f"{col}: {ids}")
        
        # --- Persistence ---
        structured_data = []
        for col, ids in outliers.items():
            for p_id in ids:
                # Retrieve the actual value for the report
                val = df_agg.loc[df_agg[id_col] == p_id, col].values[0]
                structured_data.append({id_col: p_id, 'Variable': col, 'Aggregated_Value': round(val, 2)})
        
        report_df = pd.DataFrame(structured_data)
        timestamp = datetime.now().strftime("%Y%m%d_%H%M")
        output_file = f"outlier_report_aggregated_{timestamp}.csv"
        
        report_df.to_csv(output_file, index=False)
        print(f"\nAggregated report exported to: {output_file}")
    else:
        print("\nNo outliers detected in the aggregated scores.")

if __name__ == "__main__":
    main()