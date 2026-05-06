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

def run_full_analysis():
    # Load the silver data
    df = pd.read_csv(os.path.join(SILVER_PATH, 'intro_study_silver.csv'))
    
    # Identify variable categories
    sections = {
        "CULTURE": [c for c in df.columns if "Culture_Affinity" in c],
        "PERSONALITY": [c for c in df.columns if "Personality_BFI" in c],
        "ROBOT_TRUST": [c for c in df.columns if "Robot_Trust" in c]
    }

    results = []

    for sec_name, cols in sections.items():
        for col in cols:
            it_data = df[df['Nationality'] == 'Italian'][col].dropna()
            de_data = df[df['Nationality'] == 'German'][col].dropna()
            global_data = df[col].dropna()

            if it_data.empty or de_data.empty: continue

            # 1. Statistics
            _, p_val = stats.ks_2samp(it_data, de_data)
            d, res = cliffs_delta(it_data, de_data)

            results.append({
                'Item': col,
                'IT_Mean': it_data.mean(),
                'DE_Mean': de_data.mean(),
                'P_Value': p_val,
                'Cliff_d': d,
                'Magnitude': res
            })

            # 2. SEPARATED + GLOBAL DISTRIBUTION PLOT
            plt.figure(figsize=(10, 6))
            
            if sec_name == "ROBOT_TRUST":
                # Continuous Density for 0-100%
                sns.kdeplot(global_data, color='gray', label='Global (All)', ls='--', lw=2, alpha=0.6)
                sns.kdeplot(it_data, color='#2ecc71', label='Italian', fill=True, alpha=0.4)
                sns.kdeplot(de_data, color='#3498db', label='German', fill=True, alpha=0.4)
                plt.xlim(0, 100)
            else:
                # Frequency Bars for Likert (1-5 or 1-7)
                # To show the 'Global' trend, we plot a background bar or a line
                bins = range(1, (8 if sec_name == "CULTURE" else 6))
                
                # Plot Italian and German side-by-side
                sns.countplot(data=df, x=col, hue='Nationality', 
                              palette={'Italian': '#2ecc71', 'German': '#3498db'},
                              alpha=0.8)
                
                # Overlay Global Mean as a vertical line
                plt.axvline(x=global_data.mean() - 1, color='black', ls=':', label=f'Global Mean ({global_data.mean():.2f})')

            plt.title(f"Distribution Comparison: {col.replace('_', ' ')}")
            plt.ylabel("Density / Count")
            plt.xlabel("Scale Value")
            plt.legend(frameon=True)
            
            plt.savefig(os.path.join(REPORT_PATH, f"distribution_{col}.pdf"), bbox_inches='tight')
            plt.close()

    # Save Stats Summary
    pd.DataFrame(results).to_csv(os.path.join(STATS_PATH, 'intro_stats.csv'), index=False)

if __name__ == "__main__":
    initialize()
    run_full_analysis()