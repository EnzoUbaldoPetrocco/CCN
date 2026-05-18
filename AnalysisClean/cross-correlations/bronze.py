import pandas as pd
import numpy as np
from scipy.stats import spearmanr
import os

def write_correlation_latex(df_results, filepath):
    """
    Generates a LaTeX table for statistically significant correlations.
    """
    # Filter for significant results for the table
    significant_df = df_results[df_results['Significant'] == True].copy()
    
    if significant_df.empty:
        with open(filepath, 'w') as f:
            f.write("% No significant correlations found to display.")
        return

    with open(filepath, 'w') as f:
        f.write(r"\begin{table}[ht]" + "\n")
        f.write(r"\centering" + "\n")
        f.write(r"\caption{Statistically Significant Spearman Correlations ($p < 0.05$)}" + "\n")
        f.write(r"\label{tab:meaningful_correlations}" + "\n")
        f.write(r"\begin{tabular}{llccc}" + "\n")
        f.write(r"\toprule" + "\n")
        f.write(r"Group & Trait & Outcome & $r_s$ & $p$-value \\" + "\n")
        f.write(r"\midrule" + "\n")
        
        for _, row in significant_df.iterrows():
            f.write(f"{row['Group']} & {row['Trait']} & {row['Outcome']} & {row['Correlation']} & {row['P-Value']} \\\\\n")
            
        f.write(r"\bottomrule" + "\n")
        f.write(r"\end{tabular}" + "\n")
        f.write(r"\end{table}" + "\n")

def generate_meaningful_correlations(intro_csv, paradigm_results_csv, output_path):
    # --- PATH AUDIT ---
    for path in [intro_csv, paradigm_results_csv]:
        if not os.path.exists(path):
            print(f"Error: File not found at {os.path.abspath(path)}")
            return

    # 1. Load Data
    df_intro = pd.read_csv(intro_csv)
    df_para = pd.read_csv(paradigm_results_csv)
    
    df_intro.columns = df_intro.columns.str.strip()
    df_para.columns = df_para.columns.str.strip()
    df_intro['Participant_ID'] = df_intro['Participant_ID'].astype(str).str.strip()
    df_para['Participant_ID'] = df_para['Participant_ID'].astype(str).str.strip()

    # 2. Merge
    merged_df = pd.merge(df_para, df_intro, on='Participant_ID', how='inner')
    nat_col = [c for c in merged_df.columns if 'Nationality' in c]
    target_nat_col = nat_col[0] if nat_col else None

    # Define variables
    intro_traits = ['Trust_Overall', 'Extraversion', 'Agreeableness', 
                    'Conscientiousness', 'Neuroticism', 'Openness', 'Culture_Affinity']
    outcomes = ['Culture_Closeness_Avg', 'Competence_Overall']
    
    all_findings = []

    # 3. Correlation Analysis
    groups = ['Global', 'Italian', 'German']
    for group in groups:
        if group == 'Global':
            group_df = merged_df
        elif target_nat_col:
            group_df = merged_df[merged_df[target_nat_col].astype(str).str.strip() == group]
        else:
            continue
        
        if group_df.empty: continue

        for trait in intro_traits:
            for outcome in outcomes:
                if trait in group_df.columns and outcome in group_df.columns:
                    if group_df[trait].nunique() > 1 and group_df[outcome].nunique() > 1:
                        coeff, p_val = spearmanr(group_df[trait], group_df[outcome], nan_policy='omit')
                    else:
                        coeff, p_val = 0.0, 1.0

                    if np.isnan(coeff): coeff, p_val = 0.0, 1.0
                    
                    all_findings.append({
                        'Group': group, 'Trait': trait, 'Outcome': outcome,
                        'Correlation': round(coeff, 3), 'P-Value': round(p_val, 4),
                        'Significant': p_val < 0.05
                    })

    # 4. Save Artifacts
    os.makedirs(output_path, exist_ok=True)
    results_df = pd.DataFrame(all_findings)
    results_df.to_csv(os.path.join(output_path, 'full_correlation_analysis.csv'), index=False)
    
    # NEW: Generate the LaTeX table
    write_correlation_latex(results_df, os.path.join(output_path, 'table_meaningful_correlations.tex'))

    print(f"Analysis complete. LaTeX table generated in {output_path}")

if __name__ == "__main__":
    generate_meaningful_correlations(
        '../intro/golden_layer/processed_features.csv',
          '../center/golden_layer/interaction_features_full.csv',
            './golden_layer/'
    )