import pandas as pd
import numpy as np
import os
from scipy import stats
import matplotlib.pyplot as plt
import seaborn as sns
from cliffs_delta import cliffs_delta

# --- CONFIGURATION ---
BASE_PATH = os.path.dirname(__file__)
SILVER_PATH = os.path.join(BASE_PATH, 'silver_layer')
REPORT_PATH = os.path.join(BASE_PATH, 'report_exp2')
STATS_PATH = os.path.join(BASE_PATH, 'statistics_exp2')

def initialize():
    os.makedirs(REPORT_PATH, exist_ok=True)
    os.makedirs(STATS_PATH, exist_ok=True)
    sns.set_theme(style="whitegrid", context="paper")

def get_magnitude(d):
    """Formal classification of Cliff's Delta magnitude."""
    abs_d = abs(d)
    if abs_d < 0.147: return "Negligible"
    elif abs_d < 0.33: return "Small"
    elif abs_d < 0.474: return "Medium"
    else: return "Large"

def run_stats(df, group_col, val_a, val_b, metrics, context_label):
    """Calculates KS test and Cliff's Delta for a pair of groups."""
    results = []
    for m in metrics:
        g1 = df[df[group_col] == val_a][m].dropna()
        g2 = df[df[group_col] == val_b][m].dropna()
        
        if len(g1) < 2 or len(g2) < 2: continue
        
        stat, p = stats.ks_2samp(g1, g2)
        d, res = cliffs_delta(g1, g2)
        
        results.append({
            'Context': context_label,
            'Metric': m,
            'Group_A': val_a,
            'Group_B': val_b,
            'Mean_A': g1.mean(),
            'Mean_B': g2.mean(),
            'KS_p_value': p,
            'Cliff_d': d,
            'Magnitude': get_magnitude(d)
        })
    return results

def generate_grid(df, metrics, hue_col, title, filename, x_limit):
    """Generates a consolidated grid of distribution plots."""
    num_metrics = len(metrics)
    cols = 2
    rows = (num_metrics + 1) // 2
    fig, axes = plt.subplots(rows, cols, figsize=(14, rows * 4))
    axes = axes.flatten()

    for i, m in enumerate(metrics):
        ax = axes[i]
        # Global Reference
        sns.kdeplot(df[m], ax=ax, color='gray', ls='--', label='Global', lw=1.5, alpha=0.5)
        # Grouped Data
        sns.kdeplot(data=df, x=m, hue=hue_col, ax=ax, fill=True, alpha=0.3)
        
        ax.set_title(f"Item: {m}")
        ax.set_xlim(1, x_limit)
        ax.set_xlabel("Rating")
        ax.legend(title=hue_col)

    for j in range(i + 1, len(axes)): fig.delaxes(axes[j])
    
    plt.suptitle(title, fontsize=16, y=1.02)
    plt.tight_layout()
    plt.savefig(os.path.join(REPORT_PATH, filename), bbox_inches='tight')
    plt.close()

def main():
    df = pd.read_csv(os.path.join(SILVER_PATH, 'experiment2_silver.csv'))
    
    # Define metric groups
    m_closeness = [c for c in df.columns if "Culture_Closeness" in c]
    m_competence = [c for c in df.columns if "Competence" in c]
    all_metrics = m_closeness + m_competence

    final_results = []

    # --- 1. ANALYSIS BY NATIONALITY (Global) ---
    final_results.extend(run_stats(df, 'Nationality', 'Italian', 'German', all_metrics, 'Global_Culture'))
    generate_grid(df, m_closeness, 'Nationality', 'Culture Closeness: Italian vs German', 'Grid_Closeness_Nationality.pdf', 7)
    generate_grid(df, m_competence, 'Nationality', 'Competence Assessment: Italian vs German', 'Grid_Competence_Nationality.pdf', 9)

    # --- 2. ANALYSIS BY PARADIGM (Global) ---
    for p1, p2 in [('A', 'B'), ('B', 'F'), ('A', 'F')]:
        final_results.extend(run_stats(df, 'Paradigm', p1, p2, all_metrics, 'Global_Paradigm'))
    generate_grid(df, m_closeness, 'Paradigm', 'Culture Closeness: By Paradigm', 'Grid_Closeness_Paradigm.pdf', 7)
    generate_grid(df, m_competence, 'Paradigm', 'Competence Assessment: By Paradigm', 'Grid_Competence_Paradigm.pdf', 9)

    # --- 3. INTERACTION: NATIONALITY PER PARADIGM ---
    for p in ['A', 'B', 'F']:
        sub = df[df['Paradigm'] == p]
        final_results.extend(run_stats(sub, 'Nationality', 'Italian', 'German', all_metrics, f'Nationality_within_P_{p}'))
        generate_grid(sub, m_closeness, 'Nationality', f'Closeness (Paradigm {p}): IT vs DE', f'Grid_Closeness_P_{p}.pdf', 7)

    # --- 4. INTERACTION: PARADIGM PER NATIONALITY ---
    for n in ['Italian', 'German']:
        sub = df[df['Nationality'] == n]
        for p1, p2 in [('A', 'B'), ('B', 'F'), ('A', 'F')]:
            final_results.extend(run_stats(sub, 'Paradigm', p1, p2, all_metrics, f'Paradigm_within_Nationality_{n}'))
        generate_grid(sub, m_competence, 'Paradigm', f'Competence ({n}): By Paradigm', f'Grid_Competence_{n}.pdf', 9)

    # --- EXPORT RESULTS ---
    results_df = pd.DataFrame(final_results)
    results_df.to_csv(os.path.join(STATS_PATH, 'exp2_comprehensive_stats.csv'), index=False)
    
    # Professional LaTeX Output
    with open(os.path.join(STATS_PATH, 'exp2_stats_table.tex'), 'w') as f:
        f.write(results_df.to_latex(index=False, float_format="%.3f", escape=True,
                                   caption="Comprehensive Statistical Comparison for Experiment 2",
                                   label="tab:exp2_stats"))

    print(f"Analysis complete. {len(results_df)} statistical pairs computed.")

if __name__ == "__main__":
    initialize()
    main()