import pandas as pd
import seaborn as sns
import matplotlib.pyplot as plt
import os

def generate_behavior_frequency_plot(file_path, output_folder="behavioral_plots"):
    """
    Extracts behavioral categories from a CSV and generates a separate 
    comparison figure for each category across paradigms A, B, and F.
    """
    # 1. Load and clean data
    if not os.path.exists(file_path):
        print(f"❌ Error: File {file_path} not found.")
        return

    df = pd.read_csv(file_path)
    df.columns = [c.strip() for c in df.columns]

    # Define the categories and their respective column pairs
    categories = {
        "Head Orientation": ["Head orientation", "Head orientation (optionals)"],
        "Torso Orientation": ["Torso orientation", "Torso orientation (optional)"],
        "Arms-Hands": ["Arms/Hands", "Arms/Hands (optionals)"],
        "Facial Affect": ["Facial affect", "Facial affect (optionals)"],
        "Proxemics": ["Proxemics", "Proxemics (optionals)"]
    }

    # Create output directory
    if not os.path.exists(output_folder):
        os.makedirs(output_folder)

    paradigms = ['A', 'B', 'F']

    # 2. Process each category individually
    for category_name, cols in categories.items():
        category_data = []
        
        for p in paradigms:
            sub_df = df[df['Paradigm'] == p]
            
            for col in cols:
                if col in df.columns:
                    # Explode comma-separated labels
                    series = sub_df[col].dropna().astype(str).str.split(',')
                    exploded = series.explode()
                    
                    for val in exploded:
                        val_clean = val.strip()
                        if val_clean and val_clean.lower() != 'nan':
                            category_data.append({
                                "Paradigm": p,
                                "Value": val_clean
                            })

        if not category_data:
            continue

        plot_df = pd.DataFrame(category_data)

        # 3. Create a figure for the specific category
        sns.set_theme(style="whitegrid")
        
        # FacetGrid: 1 Row, 3 Columns (A, B, F)
        g = sns.catplot(
            data=plot_df, 
            x="Value", 
            col="Paradigm", 
            kind="count",
            col_order=paradigms,
            sharex=True,  # Keeps the X-axis consistent for comparison within the category
            sharey=False, 
            height=5, 
            aspect=1.2,
            palette="viridis"
        )

        # Styling
        g.fig.suptitle(f"Frequency Analysis: {category_name}", fontsize=16, fontweight='bold', y=1.05)
        g.set_titles("Paradigm {col_name}")
        g.set_axis_labels("Observed Label", "Total Frequency")
        
        for ax in g.axes.flat:
            plt.setp(ax.get_xticklabels(), rotation=45, ha='right')

        # 4. Save the specific category plot
        safe_name = category_name.replace(" ", "_").replace("/", "-")
        save_path = os.path.join(output_folder, f"{safe_name}_comparison.png")
        g.savefig(save_path, dpi=300, bbox_inches='tight')
        plt.close()

# --- Execution ---
FILE_PATH = "./data/bronze/bronze.csv"
OUTPUT_FOLDER = "plots_04"

generate_behavior_frequency_plot(FILE_PATH, output_folder=OUTPUT_FOLDER)
print(f"✅ Success! Graphs created in /{OUTPUT_FOLDER}")