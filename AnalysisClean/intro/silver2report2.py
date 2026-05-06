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
REPORT_PATH = os.path.join(BASE_PATH, 'report_intro')
STATS_PATH = os.path.join(BASE_PATH, 'statistics_intro')

def initialize():
    os.makedirs(REPORT_PATH, exist_ok=True)
    os.makedirs(STATS_PATH, exist_ok=True)
    sns.set_theme(style="whitegrid")

def run_consolidated_report():
    # Load silver dataset
    df = pd.read_csv(os.path.join(SILVER_PATH, 'intro_study_silver.csv'))
    
    # Define Sections
    sections = {
        "Culture_Affinity": [c for c in df.columns if "Culture_Affinity" in c],
        "Personality_BFI": [c for c in df.columns if "Personality_BFI" in c],
        "Robot_Trust": [c for c in df.columns if "Robot_Trust" in c]
    }

    all_stats = []

    for section_name, cols in sections.items():
        num_cols = len(cols)
        # Create a grid: 2 columns wide, rows calculated based on number of questions
        rows = (num_cols + 1) // 2 
        fig, axes = plt.subplots(rows, 2, figsize=(15, rows * 5))
        axes = axes.flatten()

        for i, col in enumerate(cols):
            ax = axes[i]
            it_data = df[df['Nationality'] == 'Italian'][col].dropna()
            de_data = df[df['Nationality'] == 'German'][col].dropna()
            global_data = df[col].dropna()

            # 1. Statistics Calculation
            d, res = cliffs_delta(it_data, de_data)
            _, p_val = stats.ks_2samp(it_data, de_data)
            
            all_stats.append({
                'Variable': col,
                'IT_Mean': it_data.mean(),
                'DE_Mean': de_data.mean(),
                'P_Value': p_val,
                'Cliff_d': d,
                'Effect': res
            })

            # 2. Distribution Plotting (Continuous/Density Style)
            # Global distribution as reference (Gray dashed)
            sns.kdeplot(global_data, ax=ax, color='gray', ls='--', label='Global', lw=1.5)
            # Cohort distributions (Italian Green, German Blue)
            sns.kdeplot(it_data, ax=ax, color='#2ecc71', label='Italian', fill=True, alpha=0.3)
            sns.kdeplot(de_data, ax=ax, color='#3498db', label='German', fill=True, alpha=0.3)

            ax.set_title(f"Item: {col.replace('_', ' ')}")
            ax.set_xlabel("Value")
            ax.set_ylabel("Density")
            ax.legend()

            # Set limits based on scale
            if "Trust" in section_name:
                ax.set_xlim(0, 100)
            elif "Culture" in section_name:
                ax.set_xlim(1, 7)
            else:
                ax.set_xlim(1, 5)

        # Hide unused axes if number of columns is odd
        for j in range(i + 1, len(axes)):
            fig.delaxes(axes[j])

        plt.tight_layout()
        plt.savefig(os.path.join(REPORT_PATH, f"Consolidated_Distribution_{section_name}.pdf"))
        plt.close()

    # Save Stats Summary
    pd.DataFrame(all_stats).to_csv(os.path.join(STATS_PATH, 'intro_comparison_results.csv'), index=False)
    print(f"Consolidated report generated. Check {REPORT_PATH} for category-level PDFs.")

if __name__ == "__main__":
    initialize()
    run_consolidated_report()