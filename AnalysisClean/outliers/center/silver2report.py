import pandas as pd
import numpy as np
import os
import matplotlib.pyplot as plt
import seaborn as sns
from matplotlib.lines import Line2D
import matplotlib.patches as mpatches

# --- CONFIGURATION ---
BASE_PATH = os.path.dirname(__file__)
SILVER_PATH = os.path.join(BASE_PATH, 'silver_layer')
REPORT_PATH = os.path.join(BASE_PATH, 'report_exp2')

def initialize():
    if not os.path.exists(REPORT_PATH):
        os.makedirs(REPORT_PATH)
    sns.set_theme(style="whitegrid", context="paper")

def save_plot(df, metric, hue_col, context_label, x_limit):
    """
    Renders a single metric. 
    Explicitly uses legend=False to bypass the Seaborn TypeError.
    """
    # Filter out empty data to prevent further errors
    if df.empty or metric not in df.columns:
        return

    plt.figure(figsize=(7, 5))
    ax = plt.gca()
    
    # 1. Plot Global Reference (Dashed Gray)
    # We use a simple line if std is 0 to avoid KDE failure
    global_std = df[metric].std()
    if not np.isnan(global_std) and global_std > 0:
        sns.kdeplot(df[metric], ax=ax, color='gray', ls='--', lw=1.5, alpha=0.5)
    else:
        ax.axvline(df[metric].mean(), color='gray', ls='--', lw=1.5, alpha=0.5)

    # 2. Plot Grouped Data
    hue_levels = sorted(df[hue_col].unique())
    palette = sns.color_palette("tab10", n_colors=len(hue_levels))
    
    # CRITICAL: legend=False stops Seaborn from trying to iterate over the NoneType levels
    try:
        sns.kdeplot(data=df, x=metric, hue=hue_col, hue_order=hue_levels, 
                    ax=ax, fill=True, alpha=0.3, legend=False, warn_singular=False)
    except Exception:
        # If KDE still fails due to math errors, use a histogram
        sns.histplot(data=df, x=metric, hue=hue_col, hue_order=hue_levels, 
                     ax=ax, element="step", fill=True, alpha=0.3, legend=False)

    # 3. MANUAL LEGEND (This cannot fail because it doesn't use the hue_map)
    legend_items = [Line2D([0], [0], color='gray', ls='--', label='Global')]
    for i, level in enumerate(hue_levels):
        legend_items.append(mpatches.Patch(facecolor=palette[i], alpha=0.3, label=str(level)))
    
    ax.legend(handles=legend_items, title=hue_col, loc='upper right')

    # 4. Final Touch & Save
    ax.set_title(f"{metric} | {context_label}", fontsize=11, fontweight='bold')
    ax.set_xlim(1, x_limit)
    ax.set_xlabel("Rating")
    
    clean_metric = metric.replace(" ", "_").replace("/", "_")
    filename = f"{context_label}_{clean_metric}.pdf"
    
    plt.tight_layout()
    plt.savefig(os.path.join(REPORT_PATH, filename))
    plt.close()

def main():
    # Loading data from your generated inspection file
    data_file = os.path.join(SILVER_PATH, 'outlier_detailed_inspection.csv')
    if not os.path.exists(data_file):
        print(f"Error: {data_file} not found.")
        return
        
    df = pd.read_csv(data_file)
    
    m_closeness = [c for c in df.columns if "Culture_Closeness" in c]
    m_competence = [c for c in df.columns if "Competence" in c]

    # --- EXECUTION: INDIVIDUAL ITERATIONS ---
    
    # Global Nationality Analysis
    for m in m_closeness + m_competence:
        save_plot(df, m, 'Nationality', 'Global_Nationality', 7 if m in m_closeness else 9)

    # Global Paradigm Analysis
    for m in m_closeness + m_competence:
        save_plot(df, m, 'Paradigm', 'Global_Paradigm', 7 if m in m_closeness else 9)

    # Interaction: Nationality within each Paradigm
    for p in df['Paradigm'].unique():
        sub_p = df[df['Paradigm'] == p]
        for m in m_closeness:
            save_plot(sub_p, m, 'Nationality', f'Paradigm_{p}_Nationality', 7)

    # Interaction: Paradigm within each Nationality
    for n in df['Nationality'].unique():
        sub_n = df[df['Nationality'] == n]
        for m in m_competence:
            save_plot(sub_n, m, 'Paradigm', f'Nationality_{n}_Paradigm', 9)

    print(f"Success. Check the folder: {REPORT_PATH}")

if __name__ == "__main__":
    initialize()
    main()