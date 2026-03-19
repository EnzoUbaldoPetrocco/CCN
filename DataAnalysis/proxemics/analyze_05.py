import pandas as pd
import seaborn as sns
import matplotlib.pyplot as plt
import os

def generate_behavior_frequency_plot(file_path, output_folder="normalized_analysis"):
    """
    Generates a grouped bar chart for each category, normalized by the 
    total amount of labels recorded for each paradigm in that category.
    """
    # 1. Load and clean data
    if not os.path.exists(file_path):
        print(f"❌ Error: File {file_path} not found.")
        return

    df = pd.read_csv(file_path)
    df.columns = [c.strip() for c in df.columns]

    # Map categories to their column pairs
    categories = {
        "Head Orientation": ["Head orientation", "Head orientation (optionals)"],
        "Torso Orientation": ["Torso orientation", "Torso orientation (optional)"],
        "Arms-Hands": ["Arms/Hands", "Arms/Hands (optionals)"],
        "Facial Affect": ["Facial affect", "Facial affect (optionals)"],
        "Proxemics": ["Proxemics", "Proxemics (optionals)"]
    }

    if not os.path.exists(output_folder):
        os.makedirs(output_folder)

    # 2. Process each category for normalization
    for category_name, cols in categories.items():
        results_list = []
        
        for p in ['A', 'B', 'F']:
            sub_df = df[df['Paradigm'] == p]
            all_labels_for_paradigm = []
            
            for col in cols:
                if col in df.columns:
                    # Explode comma-separated strings into individual list items
                    series = sub_df[col].dropna().astype(str).str.split(',')
                    exploded = series.explode()
                    
                    for val in exploded:
                        val_clean = val.strip()
                        if val_clean and val_clean.lower() != 'nan':
                            all_labels_for_paradigm.append(val_clean)
            
            # Calculate percentages
            if all_labels_for_paradigm:
                total_count = len(all_labels_for_paradigm)
                # Count frequency of each unique label
                counts = pd.Series(all_labels_for_paradigm).value_counts()
                for label, count in counts.items():
                    results_list.append({
                        "Paradigm": p, 
                        "Behavior": label, 
                        "Percentage": (count / total_count) * 100
                    })

        if not results_list:
            continue

        plot_df = pd.DataFrame(results_list)

        # 3. Create the Normalized Grouped Plot
        plt.figure(figsize=(12, 7))
        sns.set_theme(style="whitegrid")
        
        ax = sns.barplot(
            data=plot_df, 
            x="Behavior", 
            y="Percentage", 
            hue="Paradigm", 
            hue_order=['A', 'B', 'F'],
            palette="viridis",
            edgecolor=".3"
        )

        # Formatting
        plt.title(f"{category_name}: Normalized Distribution (%)", fontsize=16, pad=20)
        plt.xlabel("Behavioral Label", fontsize=12)
        plt.ylabel("Percentage of Total Labels (%)", fontsize=12)
        plt.xticks(rotation=35, ha='right')
        plt.legend(title="Paradigm", loc='upper right')
        plt.ylim(0, 105) # Add space for labels

        # Add percentage labels on top of bars
        for container in ax.containers:
            ax.bar_label(container, fmt='%.1f%%', padding=3, fontsize=9)

        plt.tight_layout()

        # 4. Save each category separately
        safe_name = category_name.replace(" ", "_").replace("/", "-")
        save_path = os.path.join(output_folder, f"{safe_name}_Normalized.png")
        plt.savefig(save_path, dpi=300)
        plt.close()

# --- Execution ---
FILE_PATH = "./data/bronze/bronze.csv"
OUTPUT_FOLDER = "plots_05"

generate_behavior_frequency_plot(FILE_PATH, output_folder=OUTPUT_FOLDER)
print(f"✅ Success! Graphs created in /{OUTPUT_FOLDER}")