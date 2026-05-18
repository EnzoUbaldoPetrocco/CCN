import pandas as pd
import os

def extract_counting_results(input_csv):
    df = pd.read_csv(input_csv)
    # Standardize column cleaning
    df = df.loc[:, ~df.columns.str.contains('^Unnamed')]
    
    categories = ['Head orientation', 'Torso orientation', 'Arms/Hands', 'Facial affect', 'Proxemics']
    paradigms = ['B', 'F', 'A']
    
    results_main_only = []
    results_combined = []

    for cat in categories:
        # Determine the correct optional column name
        opt_col = f"{cat} (optionals)" if f"{cat} (optionals)" in df.columns else f"{cat} (optional)"
        
        for p in paradigms:
            subset = df[df['Paradigm'] == p]
            
            # --- CASE 1: Main Column Only ---
            main_counts = subset[cat].dropna().value_counts()
            for behavior, count in main_counts.items():
                results_main_only.append({'Paradigm': p, 'Feature': cat, 'Behavior': behavior, 'Count': count})
            
            # --- CASE 2: Main + Optional (Full Counting) ---
            if opt_col in df.columns:
                # Merge, split by comma, and flatten
                combined_series = (subset[cat].fillna("") + "," + subset[opt_col].fillna("")).str.split(',')
                exploded = combined_series.explode().str.strip()
                
                # CORRECTED LINE: Use .str.lower() accessor to filter out empty strings and 'nan'
                final_counts = exploded[(exploded != "") & (exploded.str.lower() != 'nan')].value_counts()
                
                for behavior, count in final_counts.items():
                    results_combined.append({'Paradigm': p, 'Feature': cat, 'Behavior': behavior, 'Count': count})

    # Convert to DataFrames
    df_main = pd.DataFrame(results_main_only)
    df_all = pd.DataFrame(results_combined)
    
    # Create directory if it doesn't exist
    os.makedirs('./silver_layer', exist_ok=True)
    
    df_main.to_csv('./silver_layer/counts_main_only.csv', index=False)
    df_all.to_csv('./silver_layer/counts_inclusive.csv', index=False)
    
    return df_main, df_all
if __name__ == "__main__":
    main_stats, inclusive_stats = extract_counting_results('./bronze_layer/bronze.csv')
    print("Files 'counts_main_only.csv' and 'counts_inclusive.csv' have been generated.")