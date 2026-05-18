import pandas as pd
import os

def generate_golden_percentage_tables(main_counts_csv, inclusive_counts_csv, output_dir):
    if not os.path.exists(output_dir):
        os.makedirs(output_dir, exist_ok=True)
    
    try:
        df_main = pd.read_csv(main_counts_csv)
        df_inc = pd.read_csv(inclusive_counts_csv)
    except FileNotFoundError as e:
        print(f"Error: {e}")
        return

    features = ['Head orientation', 'Torso orientation', 'Arms/Hands', 'Facial affect', 'Proxemics']
    # 1. Define the order here
    paradigms = ['B', 'F', 'A']

    def write_tex_percentage_file(df, filename, caption, label):
        file_path = os.path.join(output_dir, filename)
        totals = df.groupby(['Feature', 'Paradigm'])['Count'].sum().to_dict()

        with open(file_path, 'w') as f:
            f.write(r"\begin{table}[ht]" + "\n")
            f.write(r"\centering" + "\n")
            f.write(f"\\caption{{{caption}}}\n")
            f.write(f"\\label{{{label}}}\n")
            f.write(r"\begin{tabular}{llccc}" + "\n")
            f.write(r"\toprule" + "\n")
            
            # 2. DYNAMIC HEADER: This ensures the header matches the list order
            header_row = "Feature & Behavior & " + " & ".join([f"Paradigm {p} (\%)" for p in paradigms]) + r" \\" + "\n"
            f.write(header_row)
            
            f.write(r"\midrule" + "\n")
            
            for feat in features:
                feat_df = df[df['Feature'] == feat]
                unique_behaviors = sorted(feat_df['Behavior'].unique())
                
                if len(unique_behaviors) == 0: continue

                for i, beh in enumerate(unique_behaviors):
                    display_feat = feat if i == 0 else ""
                    row_values = [display_feat, beh]
                    
                    # 3. DATA ITERATION: This follows ['B', 'F', 'A']
                    for p in paradigms:
                        c_series = feat_df[(feat_df['Paradigm'] == p) & (feat_df['Behavior'] == beh)]['Count']
                        count_val = c_series.iloc[0] if not c_series.empty else 0
                        total_val = totals.get((feat, p), 0)
                        
                        if total_val > 0:
                            pct = (count_val / total_val) * 100
                            row_values.append(f"{pct:.1f}\%")
                        else:
                            row_values.append("0.0\%")
                    
                    f.write(" & ".join(row_values) + r" \\" + "\n")
                f.write(r"\midrule" + "\n")
                
            f.write(r"\bottomrule" + "\n")
            f.write(r"\end{tabular}" + "\n")
            f.write(r"\end{table}" + "\n")

    write_tex_percentage_file(df_main, 'table_main_percentages.tex', 'Main Percentages', 'tab:main_pct')
    write_tex_percentage_file(df_inc, 'table_inclusive_percentages.tex', 'Inclusive Percentages', 'tab:inc_pct')

if __name__ == "__main__":
    generate_golden_percentage_tables('./silver_layer/counts_main_only.csv', './silver_layer/counts_inclusive.csv', './golden_layer')