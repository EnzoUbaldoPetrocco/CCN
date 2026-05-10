import pandas as pd
import numpy as np
import os
import matplotlib.pyplot as plt
import seaborn as sns
from scipy.stats import chi2_contingency

# --- PERCORSI ---
BASE_PATH = os.path.dirname(__file__)
SILVER_PATH = os.path.join(BASE_PATH, 'silver_layer')
STATS_PATH = os.path.join(BASE_PATH, 'statistics_proxemics')
LATEX_PATH = os.path.join(STATS_PATH, 'latex_reports')

def cramers_v(contingency):
    chi2 = chi2_contingency(contingency)[0]
    n = contingency.sum().sum()
    r, k = contingency.shape
    return np.sqrt((chi2/n) / min(k-1, r-1))

def run_analysis():
    os.makedirs(LATEX_PATH, exist_ok=True)
    df = pd.read_csv(os.path.join(SILVER_PATH, 'proxemics_silver.csv'))
    
    stats_list = []
    categories = ['Head orientation', 'Torso orientation', 'Arms/Hands', 'Facial affect', 'Proxemics']

    for cat in categories:
        # Explode for multiple behaviors per row
        exploded = df[['Paradigm', cat]].copy()
        exploded[cat] = exploded[cat].str.split(', ')
        exploded = exploded.explode(cat)
        exploded = exploded[exploded[cat].notna() & (exploded[cat] != "nan") & (exploded[cat] != "")]
        
        # Use .values to avoid duplicate label error
        contingency = pd.crosstab(exploded['Paradigm'].values, exploded[cat].values)
        
        if contingency.size > 0:
            chi2, p, _, _ = chi2_contingency(contingency)
            v = cramers_v(contingency)
            
            stats_list.append({
                'Feature': cat,
                'Chi2': chi2,
                'p-value': p,
                'Cramers_V': v,
                'Result': 'Significant' if p < 0.05 else 'N.S.'
            })

    # --- GENERAZIONE TEX ---
    df_stats = pd.DataFrame(stats_list)
    df_stats.to_csv(os.path.join(STATS_PATH, 'proxemics_stats.csv'), index=False)

    tex_table = df_stats.to_latex(
        index=False,
        float_format="%.3f",
        caption="Chi-Square and Cramer's V Analysis for Proxemic Behaviors",
        label="tab:proxemics_stats",
        escape=True,
        column_format='lcccc'
    )

    with open(os.path.join(LATEX_PATH, 'proxemics_table.tex'), 'w') as f:
        f.write(tex_table)

    print("LaTeX table generated successfully.")

if __name__ == "__main__":
    run_analysis()