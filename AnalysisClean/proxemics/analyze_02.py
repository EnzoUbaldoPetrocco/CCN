import pandas as pd
import matplotlib.pyplot as plt
import seaborn as sns
import os

# --- 1. CONFIGURATION ---
FILE_PATH = './data/bronze/bronze.csv' 
OUTPUT_FOLDER = 'plots_02'  # The name of your subfolder
STATS_FILE = './data/processed/silver_02.csv'

def transform_and_analyze(df):
    """Handles the transformation of multi-value optional columns."""
    target_cols = [col for col in df.columns if col != 'Paradigm']
    stats_list = []
    
    for paradigm in df['Paradigm'].unique():
        p_df = df[df['Paradigm'] == paradigm]
        total_rows = len(p_df)
        
        for col in target_cols:
            if "(optional" in col.lower():
                # TRANSFORM: Split comma-separated strings and "explode" them
                # This treats "Tilted, Shaking" as two separate occurrences
                exploded = p_df[col].dropna().astype(str).str.split(',').explode().str.strip()
                counts = exploded.value_counts()
            else:
                # STANDARD: Regular categorical count
                counts = p_df[col].value_counts(dropna=False)

            # Calculate percentages relative to the total number of entries in this paradigm
            for val, count in counts.items():
                stats_list.append({
                    'Paradigm': paradigm,
                    'Category': col,
                    'Value': str(val),
                    'Count': count,
                    'Percentage': round((count / total_rows) * 100, 2)
                })
                
    return pd.DataFrame(stats_list)

# --- 2. THE PLOT PART ---
def generate_plots(stats_df, output_dir):
    """Generates and saves a chart for every behavioral category."""
    if not os.path.exists(output_dir):
        os.makedirs(output_dir)
        
    sns.set_theme(style="whitegrid", palette="muted")
    categories = stats_df['Category'].unique()

    for cat in categories:
        subset = stats_df[stats_df['Category'] == cat]
        
        # Prepare data for plotting: Values as rows, Paradigms as columns
        plot_data = subset.pivot(index='Value', columns='Paradigm', values='Percentage').fillna(0)
        
        # Create Horizontal Bar Chart (better for long labels in 'optional' columns)
        plt.figure(figsize=(12, 6))
        ax = plot_data.plot(kind='barh', width=0.8, edgecolor='white')
        
        # Formatting
        plt.title(f'Frequency Analysis: {cat}', fontsize=14, fontweight='bold', pad=20)
        plt.xlabel('Occurrence Rate (%)', fontsize=12)
        plt.ylabel('Behavior Label', fontsize=12)
        plt.legend(title='Paradigm', bbox_to_anchor=(1.05, 1), loc='upper left')
        
        # Add labels on the bars
        for container in ax.containers:
            ax.bar_label(container, fmt='%.1f%%', padding=5)

        plt.tight_layout()
        
        # Save logic
        safe_name = "".join(x for x in cat if x.isalnum() or x in " _-").strip().replace(" ", "_").lower()
        plt.savefig(os.path.join(output_dir, f"plot_{safe_name}.png"))
        plt.close('all') # Clears memory for the next plot
        print(f"✅ Generated: plot_{safe_name}.png")

# --- 3. EXECUTION ---
if __name__ == "__main__":
    try:
        raw_df = pd.read_csv(FILE_PATH)
        
        # Step 1: Transform & Analyze
        final_stats = transform_and_analyze(raw_df)
        final_stats.to_csv(STATS_FILE, index=False)
        print(f"📊 Statistics saved to {STATS_FILE}")
        
        # Step 2: Plot
        generate_plots(final_stats, OUTPUT_FOLDER)
        
        print(f"\n🚀 Success! All plots are in the /{OUTPUT_FOLDER} folder.")
    except Exception as e:
        print(f"❌ Error: {e}")