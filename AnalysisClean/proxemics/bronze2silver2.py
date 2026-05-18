import pandas as pd

def behavior_stats_to_latex(csv_file, output_tex):
    df = pd.read_csv(csv_file)
    features = ['Head orientation', 'Torso orientation', 'Arms/Hands', 'Facial affect', 'Proxemics']
    paradigms = ['B', 'F', 'A']
    
    with open(output_tex, 'w') as f:
        f.write(r"% --- Behavioral Frequency Tables ---" + "\n")
        
        for feat in features:
            # Create a pivot-style summary for this specific feature
            summary_data = {}
            
            for p in paradigms:
                subset = df[df['Paradigm'] == p]
                counts = subset[feat].value_counts(normalize=True) * 100
                summary_data[p] = counts
            
            # Combine into a single DataFrame for the table
            feat_df = pd.DataFrame(summary_data).fillna(0)
            
            f.write(r"\begin{table}[ht]" + "\n")
            f.write(r"\centering" + "\n")
            f.write(f"\\caption{{Frequency Distribution of {feat} by Paradigm (%)}}\n")
            f.write(r"\begin{tabular}{lccc}" + "\n")
            f.write(r"\toprule" + "\n")
            f.write(r"Behavior & Paradigm A & Paradigm B & Paradigm F \\" + "\n")
            f.write(r"\midrule" + "\n")
            
            for behavior, row in feat_df.iterrows():
                f.write(f"{behavior} & {row['A']:.1f}\% & {row['B']:.1f}\% & {row['F']:.1f}\% \\\\\n")
                
            f.write(r"\bottomrule" + "\n")
            f.write(r"\end{tabular}" + "\n")
            f.write(r"\end{table}" + "\n\n")

if __name__ == "__main__":
    behavior_stats_to_latex('./bronze_layer/bronze.csv', './silver_layer/video_frequencies.tex')