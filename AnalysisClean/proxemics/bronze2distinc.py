import pandas as pd
import os

def generate_distinct_golden_tables(annotations_csv, video2paradigm_csv, intro_csv, output_base):
    # Setup directories
    output_dir = os.path.join(output_base, 'distinct')
    os.makedirs(output_dir, exist_ok=True)

    # 1. Load Data
    df_ann = pd.read_csv(annotations_csv)
    df_v2p = pd.read_csv(video2paradigm_csv)
    df_intro = pd.read_csv(intro_csv)

    # --- CRITICAL: Clean all column names to remove leading/trailing spaces ---
    df_ann.columns = df_ann.columns.str.strip()
    df_v2p.columns = df_v2p.columns.str.strip()
    df_intro.columns = df_intro.columns.str.strip()

    # 2. Merge Data
    # Match the exact column names from your sample
    # ID Utente (ann) -> ID (v2p)
    # Nome del video (ann) -> Video (v2p)
    merged = pd.merge(
        df_ann, 
        df_v2p, 
        left_on=['ID Utente', 'Nome del video'], 
        right_on=['ID', 'Video']
    )
    
    # Merge with Nationality from intro.csv
    merged = pd.merge(
        merged, 
        df_intro[['Participant_ID', 'Nationality']], 
        left_on='ID Utente', 
        right_on='Participant_ID'
    )

    categories = ['Head orientation', 'Torso orientation', 'Arms/Hands', 'Facial affect', 'Proxemics']
    paradigms = ['B', 'F', 'A']
    groups = ['Italian', 'German', 'Global']

    for p_target in paradigms:
        for group in groups:
            # Filter subset
            if group == 'Global':
                subset = merged[merged['Paradigm'] == p_target]
            else:
                subset = merged[(merged['Paradigm'] == p_target) & (merged['Nationality'].str.strip() == group)]

            results = []
            for cat in categories:
                # Find columns even if they have extra spaces or (optionals)
                main_col = cat 
                opt_cols = [c for c in subset.columns if cat in c and ('optional' in c or 'optionals' in c)]
                
                if main_col in subset.columns and opt_cols:
                    opt_col = opt_cols[0]
                    # Combine and explode
                    combined = (subset[main_col].fillna("") + "," + subset[opt_col].fillna("")).str.split(',')
                    exploded = combined.explode().str.strip()
                    counts = exploded[(exploded != "") & (exploded.str.lower() != 'nan')].value_counts()
                    total = counts.sum()

                    for behavior, count in counts.items():
                        pct = (count / total * 100) if total > 0 else 0
                        # Use double backslash or raw string for LaTeX %
                        results.append({
                            'Feature': cat, 
                            'Behavior': behavior, 
                            'Percentage': f"{pct:.1f}\\%"
                        })

            # 3. Write to TeX
            if results:
                filename = f"table_{p_target}_{group.lower()}.tex"
                write_latex_distinct(results, os.path.join(output_dir, filename), p_target, group)

def write_latex_distinct(data_list, filepath, paradigm, group):
    df = pd.DataFrame(data_list)
    with open(filepath, 'w') as f:
        f.write(r"\begin{table}[ht]" + "\n\\centering\n")
        f.write(f"\\caption{{Behavioral Distribution: Paradigm {paradigm} ({group})}}\n")
        f.write(r"\begin{tabular}{llc}" + "\n\\toprule\n")
        f.write(r"Feature & Behavior & Frequency (\%) \\" + "\n\\midrule\n")
        
        last_feat = ""
        for _, row in df.iterrows():
            feat_display = row['Feature'] if row['Feature'] != last_feat else ""
            f.write(f"{feat_display} & {row['Behavior']} & {row['Percentage']} \\\\\n")
            last_feat = row['Feature']
            
        f.write(r"\bottomrule" + "\n\\end{tabular}\n\\end{table}")

if __name__ == "__main__":
    generate_distinct_golden_tables(
        './raw/annotations.csv', 
        './raw/Video2Paradigm.csv', 
        '../intro/silver_layer/intro_study_silver.csv', 
        './golden_layer'
    )