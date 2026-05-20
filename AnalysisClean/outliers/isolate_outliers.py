import pandas as pd
import os

def filter_by_outlier_ids(outliers_csv, source_csv, output_csv, filter_outliers=True):
    """
    Extracts Participant_IDs flagged as outliers and filters a target source table.
    
    Parameters:
    -----------
    outliers_csv : str
        Path to the CSV containing the outlier flags (Isolation Forest / MD).
    source_csv : str
        Path to the table you want to query/extract data from.
    output_csv : str
        Path where the filtered results should be saved.
    filter_outliers : bool, default True
        If True, extracts ONLY the outliers. 
        If False, acts as an exclusion script (keeps only clean data).
    """
    # Validate input paths
    if not os.path.exists(outliers_csv):
        print(f"Error: Outlier log not found at {outliers_csv}")
        return
    if not os.path.exists(source_csv):
        print(f"Error: Source table not found at {source_csv}")
        return

    # 1. Load outlier log and source data
    df_outliers = pd.read_csv(outliers_csv)
    df_source = pd.read_csv(source_csv)

    # Standardize spaces and data types on tracking keys
    df_outliers['Participant_ID'] = df_outliers['Participant_ID'].astype(str).str.strip()
    df_source['Participant_ID'] = df_source['Participant_ID'].astype(str).str.strip()

    # 2. Extract specific IDs flagged by Isolation Forest
    # Filters rows where Is_IsoForest_Outlier is explicitly True
    target_ids = df_outliers['Participant_ID'].unique()

    # 3. Apply Filtering Logic
    if filter_outliers:
        # Keep only the rows belonging to the outlier group
        filtered_df = df_source[df_source['Participant_ID'].isin(target_ids)]
        action_msg = f"Extracted {len(filtered_df)} rows corresponding to {len(target_ids)} outlier profiles."
    else:
        # Exclude the outliers to generate a clean baseline dataset
        filtered_df = df_source[~df_source['Participant_ID'].isin(target_ids)]
        action_msg = f"Excluded outliers. Clean dataset contains {len(filtered_df)} rows."

    # 4. Save to target location
    os.makedirs(os.path.dirname(output_csv), exist_ok=True)
    filtered_df.to_csv(output_csv, index=False)
    
    print(f"Success: {action_msg} Saved -> {output_csv}")


if __name__ == "__main__":
    # Path configuration variables
    # MULTIVARIATE
    OUTLIER_LOG = 'multivariate_outliers_20260510_1244.csv'  
    
    filter_by_outlier_ids(
        outliers_csv=OUTLIER_LOG,
        output_csv='multivariate/intro_outliers.csv',
        source_csv='../intro/golden_layer/processed_features.csv',
        filter_outliers=True
    )
    
    filter_by_outlier_ids(
        outliers_csv=OUTLIER_LOG,
        source_csv='../center/golden_layer/interaction_features_full.csv',
        output_csv='multivariate/center_outliers.csv',
        filter_outliers=True
    )

    filter_by_outlier_ids(
        outliers_csv=OUTLIER_LOG,
        output_csv='multivariate/intro_clean.csv',
        source_csv='../intro/golden_layer/processed_features.csv',
        filter_outliers=False
    )
    
    filter_by_outlier_ids(
        outliers_csv=OUTLIER_LOG,
        source_csv='../center/golden_layer/interaction_features_full.csv',
        output_csv='multivariate/center_clean.csv',
        filter_outliers=False
    )

    OUTLIER_LOG = 'outlier_report_aggregated_20260510_1225.csv'  
    filter_by_outlier_ids(
        outliers_csv=OUTLIER_LOG,
        source_csv='../intro/golden_layer/processed_features.csv',
        output_csv='univariate/intro_outliers.csv',
        filter_outliers=True  # Crucial flag inversion
    )

    filter_by_outlier_ids(
        outliers_csv=OUTLIER_LOG,
        source_csv='../center/golden_layer/interaction_features_full.csv',
        output_csv='univariate/center_outliers.csv',
        filter_outliers=True  # Crucial flag inversion
    )

    filter_by_outlier_ids(
        outliers_csv=OUTLIER_LOG,
        source_csv='../intro/golden_layer/processed_features.csv',
        output_csv='univariate/intro_clean.csv',
        filter_outliers=False  # Crucial flag inversion
    )

    filter_by_outlier_ids(
        outliers_csv=OUTLIER_LOG,
        source_csv='../center/golden_layer/interaction_features_full.csv',
        output_csv='univariate/center_clean.csv',
        filter_outliers=False  # Crucial flag inversion
    )
