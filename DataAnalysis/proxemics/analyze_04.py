import pandas as pd
import matplotlib.pyplot as plt
import seaborn as sns
import os

# --- 1. CONFIGURATION ---
FILE_PATH = './data/bronze/bronze.csv'
OUTPUT_FOLDER = 'plots_04'  # The name of your subfolder

def generate_unified_behavior_plot(path):
    df = pd.read_csv(path)
    df.columns = [c.strip() for c in df.columns]

    categories = {
        'Head': ['Head orientation', 'Head orientation (optionals)'],
        'Torso': ['Torso orientation', 'Torso orientation (optional)'],
        'Arms_Hands': ['Arms/Hands', 'Arms/Hands (optionals)'],
        'Facial_Affect': ['Facial affect', 'Facial affect (optionals)'],
        'Proxemics': ['Proxemics', 'Proxemics (optionals)']
    }

    if not os.path.exists(OUTPUT_FOLDER): os.makedirs(OUTPUT_FOLDER)
    sns.set_theme(style="white")

    for cat_name, cols in categories.items():
        valid_cols = [c for c in cols if c in df.columns]
        if not valid_cols: continue

        # Process data into a frequency count per Subject/Paradigm
        # This gives the violin the "height" and "curve" it needs
        plot_data = []
        
        # We group by Paradigm and a Window (to get a distribution, not just one number)
        for paradigm in df['Paradigm'].unique():
            p_df = df[df['Paradigm'] == paradigm].reset_index(drop=True)
            
            # Split into 10 temporal segments to see the distribution of usage
            chunks = [p_df.iloc[i:i+20] for i in range(0, len(p_df), 20)]
            
            for chunk in chunks:
                # Combine all column values in this chunk
                all_text = chunk[valid_cols].fillna('').astype(str).values.flatten()
                # Split strings that contain commas and clean
                labels = [item.strip() for sublist in [s.split(',') for s in all_text] for item in sublist]
                labels = [l for l in labels if l.lower() not in ['', 'nan', 'none']]
                
                counts = pd.Series(labels).value_counts()
                total = len(chunk)
                
                for val, count in counts.items():
                    plot_data.append({
                        'Paradigm': paradigm,
                        'Value': val,
                        'Prevalence (%)': (count / total) * 100
                    })

        if not plot_data: continue
        final_df = pd.DataFrame(plot_data)

        # Filter for the most significant values
        top_vals = final_df.groupby('Value')['Prevalence (%)'].mean().nlargest(10).index
        final_df = final_df[final_df['Value'].isin(top_vals)]

        # --- THE PLOT ---
        plt.figure(figsize=(14, 7))
        
        # This draws the Paradigms on the Y axis and the behaviors on the X axis.
        # The 'violin' shows the density of how often that label was used.
        sns.violinplot(
            data=final_df,
            x='Value',
            y='Prevalence (%)',
            hue='Paradigm',
            split=True,       # This creates the Social vs Non-Social comparison side-by-side
            inner="quart",
            palette="muted",
            bw_adjust=0.5
        )

        plt.title(f'Behavioral Profile: {cat_name}', fontsize=16, fontweight='bold')
        plt.xlabel('Annotated Values', fontsize=12)
        plt.ylabel('Annotation Density (%)', fontsize=12)
        plt.xticks(rotation=30)
        plt.legend(title='Paradigm', loc='upper right')

        plt.tight_layout()
        plt.savefig(os.path.join(OUTPUT_FOLDER, f"{cat_name}_distribution.png"))
        plt.close()

if __name__ == "__main__":
    generate_unified_behavior_plot(FILE_PATH)
    print(f"✅ Success! Graphs created in /{OUTPUT_FOLDER}")