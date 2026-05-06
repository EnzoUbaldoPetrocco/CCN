import pandas as pd
import numpy as np
import os

# --- PATH CONFIGURATION ---
BASE_PATH = os.path.dirname(__file__)
BRONZE_PATH = os.path.join(BASE_PATH, 'bronze_layer', 'online_study.csv')
SILVER_PATH = os.path.join(BASE_PATH, 'silver_layer')

def extract_metadata():
    """
    Filters the dataset to remove English participants and generates
    descriptive metadata about the remaining Italian and German cohorts.
    """
    # Load raw bronze data
    df_raw = pd.read_csv(BRONZE_PATH)

    df_raw = df_raw.drop(columns=['"Informazioni cronologiche"'],  errors='ignore')
    
    # 1. FILTERING: Remove English participants
    # Based on the header provided, 'Language/Lingua/Sprache' is at index 1
    target_languages = ['Italiano', 'Deutsch']
    df_filtered = df_raw[df_raw.iloc[:, 1].isin(target_languages)].copy()
    
    # 2. CONSOLIDATION (Metadata Fields)
    meta_df = pd.DataFrame()
    meta_df['Language'] = df_filtered.iloc[:, 1]
    
    # Helper for Prolific ID consolidation across multi-lingual columns
    def consolidate(indices):
        return df_filtered.iloc[:, indices].bfill(axis=1).iloc[:, 0]

    meta_df['Prolific_ID'] = consolidate([4, 22, 40])
    
    # 3. METADATA CALCULATIONS
    stats = {
        'Total Valid Participants': len(meta_df),
        'Italian Cohort': len(meta_df[meta_df['Language'] == 'Italiano']),
        'German Cohort': len(meta_df[meta_df['Language'] == 'Deutsch']),
    }
    
    # 4. EXPORTING RESULTS
    # Save clean metadata list to silver layer
    meta_df.to_csv(os.path.join(SILVER_PATH, 'dataset_metadata.csv'), index=False)
    
    # Text Summary for the report folder
    report_file = os.path.join(SILVER_PATH, 'metadata_summary.txt')
    with open(report_file, 'w', encoding='utf-8') as f:
        f.write("=== DATASET METADATA REPORT ===\n\n")
        for key, value in stats.items():
            f.write(f"{key}: {value}\n")
        f.write("\nNote: English-speaking participants were excluded prior to analysis.\n")

    # 5. LATEX TABLE GENERATION
    # Create a summary dataframe for the table
    summary_df = pd.DataFrame(list(stats.items()), columns=['Metric', 'Value'])
    latex_path = os.path.join(SILVER_PATH, 'metadata_table.tex')
    
    summary_df.to_latex(
        latex_path, 
        index=False, 
        caption="Summary of Dataset Composition (Excluding English)", 
        label="tab:dataset_metadata",
        escape=True
    )

    print(f"Metadata extraction complete.")
    print(f"Results saved in: {SILVER_PATH} and {SILVER_PATH}")

def main():
    try:
        extract_metadata()
    except Exception as e:
        print(f"Error during metadata extraction: {e}") 

if __name__ == "__main__":
    main()