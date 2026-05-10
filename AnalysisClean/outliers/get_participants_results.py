import pandas as pd
import numpy as np
import os

def get_participant_rows(df, target_ids, id_col='Participant_ID'):
    """
    Helper function: Returns the full rows from the dataframe 
    corresponding to the provided participant identifiers.
    """
    # Ensure target_ids is a list or similar collection
    if isinstance(target_ids, pd.Series):
        target_ids = target_ids.tolist()
        
    return df[df[id_col].isin(target_ids)].copy()

def main():
    # --- Configuration ---
    # Path to your silver data
    center_silver_path = "../center/silver_layer/experiment2_silver.csv"
    name = "center"  # Used for output naming and organization
    
    # Path to your outlier report (adjust filename to your most recent one)
    # This automatically picks the most recent file starting with 'multivariate_outliers_'
    report_files = [f for f in os.listdir('.') if f.startswith('multivariate_outliers_')]
    
    if not report_files:
        print("Error: No outlier report found in the current directory.")
        return
    
    latest_report = sorted(report_files)[-1]
    id_col = 'Participant_ID'

    # --- 1. Data Ingestion ---
    try:
        silver_df = pd.read_csv(center_silver_path)
        report_df = pd.read_csv(latest_report)
    except FileNotFoundError as e:
        print(f"Error loading files: {e}")
        return

    # --- 2. Extraction ---
    # Extracting the IDs specifically flagged in the report
    outlier_ids = report_df[id_col].unique()
    
    # Using the helper function to get full results for these participants
    anomalous_participants_df = get_participant_rows(silver_df, outlier_ids, id_col=id_col)

    # --- 3. Output/Analysis ---
    if not anomalous_participants_df.empty:
        print(f"Successfully retrieved {len(anomalous_participants_df)} rows for identified outliers.")
        
        # Displaying a subset of results for inspection
        print("\nReview of Outlier Raw Data (First 5 rows):")
        print(anomalous_participants_df.head())
        
        # Optionally save this for separate qualitative analysis
        os.makedirs(name, exist_ok=True)
        anomalous_participants_df.to_csv(f"./{name}/outlier_detailed_inspection.csv", index=False)
        print("\nDetailed outlier rows saved to 'outlier_detailed_inspection.csv'")
    else:
        print("No matches found between the report and the silver dataset.")

if __name__ == "__main__":
    main()