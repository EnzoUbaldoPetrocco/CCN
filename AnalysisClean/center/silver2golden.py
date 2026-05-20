import pandas as pd
import os
from scipy import stats
from itertools import combinations

def setup_directories():
    os.makedirs('golden_layer', exist_ok=True)

def process_interaction_data(input_csv):
    """
    Normalizes interaction data and preserves low-level features.
    """
    df = pd.read_csv(input_csv)
    
    # 1. Culture Closeness (1-7 scale)
    culture_cols = ['Culture_Closeness_Q1', 'Culture_Closeness_Q2', 'Culture_Closeness_Q3']
    for col in culture_cols:
        df[f'{col}_norm'] = (df[col] - 1) / 6
    df['Culture_Closeness_Avg'] = df[[f'{col}_norm' for col in culture_cols]].mean(axis=1)

    # 2. Competence (1-9 scale normalized to 0-1)
    comp_cols = [f'Competence_Q{i}' for i in range(1, 7)]
    for col in comp_cols:
        df[f'{col}_norm'] = (df[col] - 1) / 8
        
    df['Competence_Overall'] = df[[f'{col}_norm' for col in comp_cols]].mean(axis=1)

    all_cols = culture_cols + comp_cols
    df['all_average'] = df[['Culture_Closeness_Avg', 'Competence_Overall']].mean(axis=1)
    return df

def run_global_paradigm_comparison(df):
    """
    Pools all nationalities together to compare Paradigms globally.
    """
    features = ['Culture_Closeness_Avg', 'Competence_Overall', 'all_average']
    results = []
    paradigms = df['Paradigm'].unique()
    
    for p1, p2 in combinations(paradigms, 2):
        for feat in features:
            g1 = df[df['Paradigm'] == p1][feat].dropna()
            g2 = df[df['Paradigm'] == p2][feat].dropna()
            
            if len(g1) > 0 and len(g2) > 0:
                u_stat, p_val = stats.mannwhitneyu(g1, g2, alternative='two-sided')
                results.append({
                    'Comparison': f'{p1} vs {p2}',
                    'Feature': feat,
                    'Mean1': g1.mean(),
                    'Std1': g1.std(),
                    'Mean2': g2.mean(),
                    'Std2': g2.std(),
                    'U_Stat': u_stat,
                    'p_Value': p_val
                })
    return pd.DataFrame(results)

def export_global_latex(res_df):
    """
    Exports the Global (Mixed Nationality) comparison to a formatted LaTeX table.
    """
    formatted_data = []
    col_g1 = r'Group 1 $\mu \pm \sigma$'
    col_g2 = r'Group 2 $\mu \pm \sigma$'

    for _, row in res_df.iterrows():
        formatted_data.append({
            'Comparison': row['Comparison'],
            'Feature': row['Feature'],
            col_g1: fr"${row['Mean1']:.5f} \pm {row['Std1']:.5f}$",
            col_g2: fr"${row['Mean2']:.5f} \pm {row['Std2']:.5f}$",
            r'$U$ Stat': f"{row['U_Stat']:.5f}",
            r'$p$-value': f"{row['p_Value']:.5f}",
            'Sign.': 'Yes' if row['p_Value'] < 0.05 else 'No'
        })
    
    tex_df = pd.DataFrame(formatted_data)
    output_path = 'golden_layer/global_paradigm_table.tex'
    
    with open(output_path, 'w') as f:
        f.write(tex_df.to_latex(index=False, escape=False, 
                                 column_format='llccccc',
                                 caption="Global Paradigm Comparison (Italians + Germans Combined)",
                                 label="tab:global_paradigms"))
        
def golden2tables(golden_df: pd.DataFrame):
    """
    Generates a descriptive statistics table (Mean and Std) for every 
    Nationality and Paradigm combination.
    """
    features = ['Culture_Closeness_Avg', 'Competence_Overall', 'all_average']
    df_stats = []
    
    nationalities = golden_df['Nationality'].unique()
    paradigms = sorted(golden_df['Paradigm'].unique())

    for nat in nationalities:
        nat_data = golden_df[golden_df['Nationality'] == nat]
        for p in paradigms:
            p_data = nat_data[nat_data['Paradigm'] == p]
            
            if not p_data.empty:
                row = {
                    'Nationality': nat,
                    'Paradigm': p
                }
                for feat in features:
                    mean = p_data[feat].mean()
                    std = p_data[feat].std()
                    
                    row[f'{feat}_Mean'] = mean
                    row[f'{feat}_Std'] = std
                    # Replaces underscore with space dynamically
                    row[feat.replace('_', ' ')] = fr"${mean:.3f} \pm {std:.3f}$"
                
                df_stats.append(row)

    summary_df = pd.DataFrame(df_stats)
    summary_df.to_csv('golden_layer/descriptive_stats_by_nationality.csv', index=False)

    # FIXED: 'all_average' changed to 'all average' to match string translation keys
    latex_cols = ['Nationality', 'Paradigm', 'Culture Closeness Avg', 'Competence Overall', 'all average']
    tex_df = summary_df[latex_cols]

    output_path = 'golden_layer/nationality_descriptive_table.tex'
    with open(output_path, 'w') as f:
        # Adjusted column format to 'llccc' to fit all 5 output columns appropriately
        f.write(tex_df.to_latex(index=False, escape=False, 
                                 column_format='llccc',
                                 caption="Descriptive Statistics by Nationality and Paradigm",
                                 label="tab:nat_desc_stats"))
    
    print(f"Success: Descriptive tables generated for {list(nationalities)}")

if __name__ == "__main__":
    setup_directories()
    input_path = 'silver_layer/experiment2_silver.csv'
    
    if os.path.exists(input_path):
        golden_df = process_interaction_data(input_path)
        golden_df.to_csv('golden_layer/interaction_features_full.csv', index=False)
        
        global_stats = run_global_paradigm_comparison(golden_df)
        global_stats.to_csv('golden_layer/global_paradigm_stats.csv', index=False)
        
        export_global_latex(global_stats)
        golden2tables(golden_df)
        
        print("Success: All analysis generated in golden_layer.")
    else:
        print("Error: silver_layer/experiment2_silver.csv not found.")