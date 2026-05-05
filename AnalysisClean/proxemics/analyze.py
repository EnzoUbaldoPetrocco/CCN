import pandas as pd
import matplotlib.pyplot as plt
import seaborn as sns
import os  # Added to handle folder creation

# 1. Configuration
FILE_PATH = './data/bronze/bronze.csv' 
OUTPUT_FOLDER = 'plots'  # The name of your subfolder
OUTPUT_STATS = './data/processed/silver.csv'

def perform_behavioral_analysis(path):
    # Ensure the 'plots' subfolder exists
    if not os.path.exists(OUTPUT_FOLDER):
        os.makedirs(OUTPUT_FOLDER)
        print(f"📁 Created folder: {OUTPUT_FOLDER}")

    # Load data
    df = pd.read_csv(path)
    target_cols = [col for col in df.columns if col != 'Paradigm']
    stats_list = []
    
    # 2. Extract Statistics
    for paradigm in df['Paradigm'].unique():
        p_df = df[df['Paradigm'] == paradigm]
        for col in target_cols:
            counts = p_df[col].value_counts(dropna=False)
            percents = p_df[col].value_counts(normalize=True, dropna=False) * 100
            for val, count in counts.items():
                stats_list.append({
                    'Paradigm': paradigm,
                    'Category': col,
                    'Value': str(val),
                    'Count': count,
                    'Percentage': round(percents[val], 2)
                })
    
    stats_df = pd.DataFrame(stats_list)
    stats_df.to_csv(OUTPUT_STATS, index=False)
    
    # 3. Generate Graphics
    sns.set_theme(style="whitegrid")
    
    for category in target_cols:
        subset = stats_df[stats_df['Category'] == category]
        plot_data = subset.pivot(index='Value', columns='Paradigm', values='Percentage').fillna(0)
        
        ax = plot_data.plot(kind='bar', figsize=(12, 7), width=0.8)
        
        plt.title(f'Distribution: {category}', fontsize=16, fontweight='bold')
        plt.ylabel('Percentage (%)')
        plt.xlabel('Behavioral Value')
        plt.xticks(rotation=45, ha='right')
        plt.legend(title='Paradigm', bbox_to_anchor=(1.05, 1), loc='upper left')
        
        for container in ax.containers:
            ax.bar_label(container, fmt='%.1f%%', padding=3, fontsize=9)
            
        plt.tight_layout()
        
        # Clean name and save INSIDE the subfolder
        clean_name = category.replace("/", "_").replace(" ", "_").replace("(", "").replace(")", "").lower()
        save_path = os.path.join(OUTPUT_FOLDER, f"chart_{clean_name}.png")
        
        plt.savefig(save_path)
        plt.close()
        print(f"📊 Saved: {save_path}")

if __name__ == "__main__":
    try:
        perform_behavioral_analysis(FILE_PATH)
        print(f"\n🚀 Done! Statistics in {OUTPUT_STATS} and plots in the /{OUTPUT_FOLDER} folder.")
    except Exception as e:
        print(f"❌ Error: {e}")