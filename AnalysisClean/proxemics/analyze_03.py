import pandas as pd
import matplotlib.pyplot as plt
import seaborn as sns
import os

# --- 1. SETTINGS ---
FILE_PATH = './data/bronze/bronze.csv' 
OUTPUT_FOLDER = 'plots_03'
STATS_FILE = './data/processed/silver_03.csv'

def run_normalized_analysis(df):
    df.columns = [col.strip() for col in df.columns]
    stats_list = []
    
    mapping = {
        'Head_Behavior': ['Head orientation', 'Head orientation (optionals)'],
        'Torso_Behavior': ['Torso orientation', 'Torso orientation (optional)'],
        'Arms_Hands_Behavior': ['Arms/Hands', 'Arms/Hands (optionals)'],
        'Facial_Affect_Behavior': ['Facial affect', 'Facial affect (optionals)'],
        'Proxemics_Behavior': ['Proxemics', 'Proxemics (optionals)']
    }

    for paradigm in df['Paradigm'].unique():
        p_df = df[df['Paradigm'] == paradigm]
        
        for group_name, cols in mapping.items():
            pool = []
            for col in cols:
                if col in p_df.columns:
                    items = p_df[col].dropna().astype(str).str.split(',').explode().str.strip()
                    items = items[items.map(lambda x: x.lower() not in ['nan', 'none', ''])]
                    pool.extend(items.tolist())

            # 1. Get raw counts
            counts = pd.Series(pool).value_counts()
            
            # 2. NORMALIZE: Divide by the sum of ALL behaviors found in this category
            total_behavior_count = counts.sum()
            
            if total_behavior_count > 0:
                for val, count in counts.items():
                    stats_list.append({
                        'Paradigm': paradigm,
                        'Group': group_name,
                        'Behavior': val,
                        'Raw_Count': count,
                        # This now sums to 100% across all behaviors in the group
                        'Normalized_Pct': round((count / total_behavior_count) * 100, 2)
                    })
                
    return pd.DataFrame(stats_list)

# --- 2. THE PLOT PART ---
def generate_normalized_plots(stats_df, output_dir):
    if not os.path.exists(output_dir): os.makedirs(output_dir)
    sns.set_theme(style="whitegrid")

    for group in stats_df['Group'].unique():
        subset = stats_df[stats_df['Group'] == group]
        plot_data = subset.pivot(index='Behavior', columns='Paradigm', values='Normalized_Pct').fillna(0)
        
        # Plotting
        ax = plot_data.plot(kind='barh', figsize=(10, 7), width=0.8)
        plt.title(f'Normalized Distribution: {group} (Sum = 100%)', fontweight='bold')
        plt.xlabel('Percentage of Total Observations (%)')
        
        for container in ax.containers:
            ax.bar_label(container, fmt='%.1f%%', padding=5)

        plt.tight_layout()
        plt.savefig(os.path.join(output_dir, f"normalized_{group.lower()}.png"))
        plt.close()

if __name__ == "__main__":
    try:
        raw_df = pd.read_csv(FILE_PATH)
        final_stats = run_normalized_analysis(raw_df)
        
        os.makedirs(os.path.dirname(STATS_FILE), exist_ok=True)
        final_stats.to_csv(STATS_FILE, index=False)
        
        generate_normalized_plots(final_stats, OUTPUT_FOLDER)
        print("✅ Success! Charts are now normalized to 100%.")
    except Exception as e:
        print(f"❌ Error: {e}")