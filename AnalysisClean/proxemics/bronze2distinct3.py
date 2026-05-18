import pandas as pd
import os

def generate_full_comparative_suite(annotations_csv, video2paradigm_csv, intro_csv, output_base):
    # Setup directories
    output_dir = os.path.join(output_base, 'distinct')
    os.makedirs(output_dir, exist_ok=True)

    # 1. Load and Clean Data
    df_ann = pd.read_csv(annotations_csv)
    df_v2p = pd.read_csv(video2paradigm_csv)
    df_intro = pd.read_csv(intro_csv)

    for df in [df_ann, df_v2p, df_intro]:
        df.columns = df.columns.str.strip()
    
    df_ann['ID Utente'] = df_ann['ID Utente'].str.strip()
    df_v2p['ID'] = df_v2p['ID'].str.strip()
    df_intro['Participant_ID'] = df_intro['Participant_ID'].str.strip()
    df_intro['Nationality'] = df_intro['Nationality'].str.strip()

    # 2. Relational Merge
    merged = pd.merge(df_ann, df_v2p, left_on=['ID Utente', 'Nome del video'], right_on=['ID', 'Video'])
    merged = pd.merge(merged, df_intro[['Participant_ID', 'Nationality']], left_on='ID Utente', right_on='Participant_ID')

    categories = ['Head orientation', 'Torso orientation', 'Arms/Hands', 'Facial affect', 'Proxemics']
    paradigms = ['B', 'F', 'A']
    groups = ['Global', 'Italian', 'German']

    for group in groups:
        group_df = merged if group == 'Global' else merged[merged['Nationality'] == group]
        if group_df.empty: continue

        # Generate both Analysis Types: Main and Inclusive
        for mode in ['Main', 'Inclusive']:
            table_data = []

            for cat in categories:
                main_col = cat
                opt_col = [c for c in group_df.columns if cat in c and ('optional' in c or 'optionals' in c)][0]
                
                # Determine all behaviors present in this category for the group
                if mode == 'Inclusive':
                    combined_all = (group_df[main_col].fillna("") + "," + group_df[opt_col].fillna("")).str.split(',')
                else:
                    combined_all = group_df[main_col].fillna("").astype(str).str.split(',')

                all_behaviors = sorted(set([b.strip() for sublist in combined_all for b in sublist if b.strip() and b.strip().lower() != 'nan']))

                for behavior in all_behaviors:
                    row = {'Feature': cat, 'Behavior': behavior}
                    for p in paradigms:
                        subset_p = group_df[group_df['Paradigm'] == p]
                        
                        if mode == 'Inclusive':
                            p_combined = (subset_p[main_col].fillna("") + "," + subset_p[opt_col].fillna("")).str.split(',')
                        else:
                            p_combined = subset_p[main_col].fillna("").astype(str).str.split(',')

                        p_exploded = p_combined.explode().str.strip()
                        p_counts = p_exploded[(p_exploded != "") & (p_exploded.str.lower() != 'nan')].value_counts()
                        
                        total = p_counts.sum()
                        count = p_counts.get(behavior, 0)
                        pct = (count / total * 100) if total > 0 else 0
                        row[f'Paradigm_{p}'] = f"{pct:.1f}\%"
                    
                    table_data.append(row)

            # 3. Write TeX
            filename = f"table_{mode.lower()}_{group.lower()}.tex"
            caption = f"{mode} Percentages: {group} Results"
            label = f"tab:{mode.lower()}_pct_{group.lower()}"
            write_comparative_latex(table_data, os.path.join(output_dir, filename), caption, label, paradigms)

def write_comparative_latex(data_list, filepath, caption, label, paradigms):
    df = pd.DataFrame(data_list)
    with open(filepath, 'w') as f:
        f.write(r"\begin{table}[ht]" + "\n\\centering\n")
        f.write(f"\\caption{{{caption}}}\n")
        f.write(f"\\label{{{label}}}\n")
        f.write(r"\begin{tabular}{llccc}" + "\n\\toprule\n")
        header = "Feature & Behavior & " + " & ".join([f"Paradigm {p} (\%)" for p in paradigms]) + r" \\" + "\n"
        f.write(header + r"\midrule" + "\n")
        
        last_feat = ""
        for _, row in df.iterrows():
            if row['Feature'] != last_feat:
                if last_feat != "": f.write(r"\midrule" + "\n")
                feat_display = row['Feature']
            else:
                feat_display = "~"
            
            f.write(f"{feat_display} & {row['Behavior']} & {row['Paradigm_B']} & {row['Paradigm_F']} & {row['Paradigm_A']} \\\\\n")
            last_feat = row['Feature']
            
        f.write(r"\bottomrule" + "\n\\end{tabular}\n\\end{table}")

if __name__ == "__main__":
    generate_full_comparative_suite(
        './raw/annotations.csv', 
        './raw/Video2Paradigm.csv', 
        '../intro/silver_layer/intro_study_silver.csv', 
        './golden_layer'
    )