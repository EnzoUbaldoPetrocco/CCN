import pandas as pd
import os
from scipy import stats

def setup_directories():
    """Ensures the Medallion Architecture folders exist."""
    os.makedirs('golden_layer', exist_ok=True)

def extract_features(input_file):
    """
    Loads silver layer data and extracts normalized high-level features.
    """
    df = pd.read_csv(input_file)
    
    # 1. Trust Perception (14-item mean, normalized to 0-1 range)
    trust_cols = [f'Robot_Trust_Q{i}' for i in range(1, 15)]
    df['Trust_Overall'] = df[trust_cols].mean(axis=1) / 100

    # 2. OCEAN Personality Traits (BFI-10 Mapping, normalized to 0-1 range)
    # Formula: (Average - Min_Scale) / (Max_Scale - Min_Scale) -> (mean - 1) / 4
    df['Extraversion'] = (df[['Personality_BFI_Q1', 'Personality_BFI_Q6']].mean(axis=1) - 1) / 4
    df['Agreeableness'] = (df[['Personality_BFI_Q2', 'Personality_BFI_Q7']].mean(axis=1) - 1) / 4
    df['Conscientiousness'] = (df[['Personality_BFI_Q3', 'Personality_BFI_Q8']].mean(axis=1) - 1) / 4
    df['Neuroticism'] = (df[['Personality_BFI_Q4', 'Personality_BFI_Q9']].mean(axis=1) - 1) / 4
    df['Openness'] = (df[['Personality_BFI_Q5', 'Personality_BFI_Q10']].mean(axis=1) - 1) / 4

    # 3. Cultural Affinity (3-item mean, 1-7 scale)
    culture_cols = ['Culture_Affinity_Q1', 'Culture_Affinity_Q2', 'Culture_Affinity_Q3']
    df['Culture_Affinity'] = (df[culture_cols].mean(axis=1) - 1) / 6

    # Return only the Golden Features
    golden_cols = ['Nationality', 'Participant_ID', 'Trust_Overall', 'Extraversion', 
                   'Agreeableness', 'Conscientiousness', 'Neuroticism', 'Openness', 'Culture_Affinity']
    return df[golden_cols]

def perform_statistical_analysis(df):
    """
    Performs descriptive statistics and Mann-Whitney U tests for non-parametric comparison.
    """
    features = ['Trust_Overall', 'Extraversion', 'Agreeableness', 
                'Conscientiousness', 'Neuroticism', 'Openness', 'Culture_Affinity']

    # Calculate Descriptive Statistics (Mean and Std Dev)
    stats_german = df[df['Nationality'] == 'German'][features].agg(['mean', 'std']).T.add_prefix('German_')
    stats_italian = df[df['Nationality'] == 'Italian'][features].agg(['mean', 'std']).T.add_prefix('Italian_')
    
    # Merge statistics
    comparison_table = pd.concat([stats_german, stats_italian], axis=1)

    # Inferential Statistics: Mann-Whitney U Test (Non-Parametric)
    u_stats, p_values = [], []
    
    for feat in features:
        g_group = df[df['Nationality'] == 'German'][feat]
        i_group = df[df['Nationality'] == 'Italian'][feat]
        
        # Mann-Whitney U test is preferred for small, non-normal samples
        u_val, p_val = stats.mannwhitneyu(g_group, i_group, alternative='two-sided')
        
        u_stats.append(u_val)
        p_values.append(p_val)

    comparison_table['U_Statistic'] = u_stats
    comparison_table['p_Value'] = p_values
    
    # Determine Statistical Relevance (Alpha = 0.05)
    comparison_table['Significant'] = comparison_table['p_Value'].apply(
        lambda x: 'Yes ($p < 0.05$)' if x < 0.05 else 'No'
    )
    
    return comparison_table

def export_to_latex(results_df):
    """
    Converts results to the specific LaTeX format: mu \pm \sigma.
    """
    # Create a new list to store rows for the formatted table
    formatted_rows = []
    
    for index, row in results_df.iterrows():
        formatted_rows.append({
            'Feature': index,
            'German $\\mu \\pm \\sigma$': f"${row['German_mean']:.5f} \\pm {row['German_std']:.5f}$",
            'Italian $\\mu \\pm \\sigma$': f"${row['Italian_mean']:.5f} \\pm {row['Italian_std']:.5f}$",
            '$U$ Stat': f"{row['U_Statistic']:.5f}",
            '$p$-value': f"{row['p_Value']:.5f}",
            'Sign.': 'Yes' if row['p_Value'] < 0.05 else 'No'
        })
    
    # Create the final DataFrame for export
    df_tex = pd.DataFrame(formatted_rows)
    
    output_path = 'golden_layer/statistical_table.tex'
    with open(output_path, 'w') as f:
        # column_format 'lccccc' matches your desired 6 columns
        f.write(df_tex.to_latex(index=False, 
                                 column_format='lccccc', 
                                 escape=False, 
                                 caption="Descriptive and Inferential Statistics (Mann-Whitney U)",
                                 label="tab:significance_stats"))
    
    print(f"LaTeX table generated successfully at: {output_path}")

if __name__ == "__main__":
    # Define file paths
    input_csv = 'silver_layer/intro_study_silver.csv'
    
    setup_directories()

    if os.path.exists(input_csv):
        # 1. Feature Extraction (Silver -> Golden)
        golden_data = extract_features(input_csv)
        golden_data.to_csv('golden_layer/processed_features.csv', index=False)
        print("Golden features extracted and saved.")

        # 2. Statistical Analysis
        stats_results = perform_statistical_analysis(golden_data)
        stats_results.to_csv('golden_layer/statistical_comparison.csv')
        print("Statistical analysis complete.")

        # 3. LaTeX Export
        export_to_latex(stats_results)
    else:
        print(f"Critical Error: {input_csv} not found. Please check your silver_layer folder.")