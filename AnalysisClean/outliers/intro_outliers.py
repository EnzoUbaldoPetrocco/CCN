import pandas as pd
import numpy as np
from datetime import datetime
import os

def get_outlier_participant_ids(df, id_col='Participant_ID', threshold=3.0):
    """
    Identifies Participant IDs associated with outliers and returns a structured DataFrame.
    """
    all_outliers = []
    
    numeric_cols = df.select_dtypes(include=[np.number]).columns
    cols_to_analyze = [col for col in numeric_cols if col != id_col]
    
    for col in cols_to_analyze:
        series = df[col]
        median = series.median()
        mad = (series - median).abs().median()
        
        if mad == 0:
            continue
            
        # Modified Z-score calculation
        mod_z = 0.6745 * (series - median) / mad
        mask = mod_z.abs() > threshold
        
        if mask.any():
            temp_df = df.loc[mask, [id_col, col]].copy()
            temp_df.columns = [id_col, 'Observed_Value']
            temp_df['Variable'] = col
            temp_df['Modified_Z_Score'] = mod_z[mask]
            all_outliers.append(temp_df)
            
    if all_outliers:
        return pd.concat(all_outliers, ignore_index=True)
    return pd.DataFrame()

def main():
    # --- Configuration ---
    input_file = "../intro/silver_layer/intro_study_silver.csv"
    id_column_name = 'Participant_ID'  # Defined here to prevent NameError
    threshold_value = 2.0
    
    timestamp = datetime.now().strftime("%Y%m%d_%H%M")
    output_file = f"outlier_report_{timestamp}.csv"

    if not os.path.exists(input_file):
        print(f"Error: File not found at {input_file}")
        return

    try:
        df = pd.read_csv(input_file)
    except Exception as e:
        print(f"Error reading CSV: {e}")
        return

    # Execute detection
    outlier_df = get_outlier_participant_ids(df, id_col=id_column_name, threshold=threshold_value)
    
    if not outlier_df.empty:
        # Save structured results
        outlier_df.to_csv(output_file, index=False)
        print(f"Analysis Complete. {len(outlier_df)} outlier instances recorded.")
        print(f"Results saved to: {output_file}")
        
        # Display summary using the local variable
        print("\nSummary of Outliers per Variable:")
        print(outlier_df.groupby('Variable')[id_column_name].count())
    else:
        print("\nNo outliers detected. Adjust threshold if necessary.")

if __name__ == "__main__":
    main()