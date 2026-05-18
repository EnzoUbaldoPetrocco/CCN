import pandas as pd
import os

def generate_distinct_golden_artifacts(annotations_csv, video2paradigm_csv, intro_csv, output_base):
    # Setup the 'distinct' directory
    output_dir = os.path.join(output_base, 'distinct')
    os.makedirs(output_dir, exist_ok=True)

    # 1. Load Data
    df_ann = pd.read_csv(annotations_csv)
    df_v2p = pd.read_csv(video2paradigm_csv)
    df_intro = pd.read_csv(intro_csv)

    # Data Cleaning for Merge Keys
    df_ann['ID Utente'] = df_ann['ID Utente'].str.strip()
    df_v2p['ID'] = df_v2p['ID'].str.strip()
    df_intro['Participant_ID'] = df_intro['Participant_ID'].str.strip()
    df_intro['Nationality'] = df_intro['Nationality'].str.strip()

    # 2. Relational Merge
    merged = pd.merge(
        df_ann, 
        df_v2p, 
        left_on=['ID Utente', 'Nome del video'], 
        right_on=['ID', 'Video']
    )
    
    merged = pd.merge(
        merged, 
        df_intro[['Participant_ID', 'Nationality']], 
        left_on='ID Utente', 
        right_on='Participant_ID'
    )

    # Define exact column names for counting logic
    categories = [
        ('Head orientation ', 'Head orientation (optionals)'),
        ('Torso orientation ', 'Torso orientation (optional)'),
        ('Arms/Hands ', 'Arms/Hands (optionals)'),
        ('Facial affect ', 'Facial affect (optionals)'),
        ('Proxemics ', 'Proxemics (optionals)')
    ]
    
    paradigms = ['B', 'F', 'A']
    groups = ['Italian', 'German', 'Global']

    for p_target in paradigms:
        for group in groups:
            # --- FILTERING ---
            if group == 'Global':
                subset = merged[merged['Paradigm'] == p_target]
            else:
                subset = merged[(merged['Paradigm'] == p_target) & (merged['Nationality'] == group)]

            if subset.empty:
                continue

            subset = subset.drop(columns=['ID Utente', 'Video', 'Participant_ID', 'Io sono', 'Informazioni cronologiche', 'Nome del video', 'ID'], errors='ignore')

            # --- SAVE FILTERED CSV ---
            # This creates the CSV files you were looking for
            csv_filename = f"data_{p_target}_{group.lower()}.csv"
            subset.to_csv(os.path.join(output_dir, csv_filename), index=False)

            # --- CALCULATE PERCENTAGES FOR TEX ---
            results = []
            for main_col, opt_col in categories:
                if main_col in subset.columns and opt_col in subset.columns:
                    # Inclusive logic (Primary + Optional)
                    combined = (subset[main_col].fillna("") + "," + subset[opt_col].fillna("")).str.split(',')
                    exploded = combined.explode().str.strip()
                    counts = exploded[(exploded != "") & (exploded.str.lower() != 'nan')].value_counts()
                    total = counts.sum()

                    for behavior, count in counts.items():
                        pct = (count / total * 100) if total > 0 else 0
                        results.append({
                            'Feature': main_col.strip(), 
                            'Behavior': behavior, 
                            'Percentage': f"{pct:.1f}\\%"
                        })

            # --- SAVE TEX TABLE ---
            if results:
                tex_filename = f"table_{p_target}_{group.lower()}.tex"
                write_latex_distinct(results, os.path.join(output_dir, tex_filename), p_target, group)

def write_latex_distinct(data_list, filepath, paradigm, group):
    df = pd.DataFrame(data_list)
    with open(filepath, 'w') as f:
        f.write(r"\begin{table}[ht]" + "\n\\centering\n")
        f.write(f"\\caption{{Behavioral Frequency Distribution: Paradigm {paradigm} ({group})}}\n")
        f.write(r"\begin{tabular}{llc}" + "\n\\toprule\n")
        f.write(r"Feature & Behavior & Frequency (\%) \\" + "\n\\midrule\n")
        
        last_feat = ""
        for _, row in df.iterrows():
            feat_display = row['Feature'] if row['Feature'] != last_feat else ""
            f.write(f"{feat_display} & {row['Behavior']} & {row['Percentage']} \\\\\n")
            last_feat = row['Feature']
            
        f.write(r"\bottomrule" + "\n\\end{tabular}\n\\end{table}")

if __name__ == "__main__":
    generate_distinct_golden_artifacts(
        './raw/annotations.csv', 
        './raw/Video2Paradigm.csv', 
        '../intro/silver_layer/intro_study_silver.csv', 
        './golden_layer'
    )
    print("Filtered CSVs and TeX tables generated in ./golden_layer/distinct/")