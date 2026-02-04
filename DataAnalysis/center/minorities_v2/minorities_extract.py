import pandas as pd
import json
import os

def process_and_save_results(input_json_path, output_prefix="experiment_results"):
    if not os.path.exists(input_json_path):
        return f"File not found: {input_json_path}"

    try:
        with open(input_json_path, 'r') as f:
            json_data = json.load(f)
            
        rows = []
        for entry in json_data:
            if "Participant ID" not in entry: continue
            indices = entry["Participant ID"].keys()
            for idx in indices:
                rows.append({
                    "Closeness": entry["mean_cultural_closeness"][idx],
                    "Competence": entry["mean_competence"][idx],
                    "Condition": entry["P"][idx],
                    "Source": entry["outlier_source"][idx]
                })

        df = pd.DataFrame(rows)

        # Filter out the unwanted source
        df = df[df["Source"] != "mean_personality_subj"]

        # 1. Normalize Likert scales
        df["Closeness"] = (df["Closeness"] - 1) / 6
        df["Competence"] = (df["Competence"] - 1) / 8

        # 2. Calculate Averages AND Sample Sizes (n)
        target_cols = ["Closeness", "Competence"]
        
        # We group and aggregate both mean and count
        summary = df.groupby(["Source", "Condition"])[target_cols].agg(['mean', 'size'])
        
        # Flatten the column names (e.g., ('Closeness', 'mean') -> 'Closeness')
        # And keep the 'size' from one of the columns as our sample count 'n'
        n_counts = summary[('Closeness', 'size')] 
        summary = summary.xs('mean', axis=1, level=1)
        summary['n'] = n_counts
        
        # Calculate Grand Average
        summary["Grand_Average"] = summary[target_cols].mean(axis=1)

        # 3. Save raw summary as JSON
        summary.reset_index().to_json(f"{output_prefix}_summary.json", orient="records", indent=4)

        # 4. Generate LaTeX Table
        latex_lines = [
            "\\begin{table}[ht]",
            "    \\centering",
            "    \\caption{Percentage Results with Sample Sizes ($n$): " + output_prefix.replace('_', ' ') + "}",
            "    \\begin{tabular}{l l c c c c}", # Added one 'c' for n column
            "        \\toprule",
            "        \\textbf{Outlier Source} & \\textbf{Cond.} & \\textbf{$n$} & \\textbf{Closeness} & \\textbf{Competence} & \\textbf{Average} \\\\",
            "        \\midrule"
        ]

        cond_order = {"B": 0, "F": 1, "A": 2}
        summary = summary.reset_index()
        summary['sort_key'] = summary['Condition'].map(cond_order)
        summary = summary.sort_values(['Source', 'sort_key'])

        current_source = None
        for _, row in summary.iterrows():
            if row['Source'] != current_source:
                if current_source is not None:
                    latex_lines.append("        \\addlinespace")
                source_display = row['Source'].replace('_', '\\_')
                current_source = row['Source']
            else:
                source_display = ""

            # Formatting
            n_val = int(row['n'])
            p_close = f"{row['Closeness'] * 100:.1f}\\%" if pd.notnull(row['Closeness']) else "N/A"
            p_comp = f"{row['Competence'] * 100:.1f}\\%" if pd.notnull(row['Competence']) else "N/A"
            p_avg = f"{row['Grand_Average'] * 100:.1f}\\%" if pd.notnull(row['Grand_Average']) else "N/A"

            line = f"        {source_display} & {row['Condition']} & {n_val} & {p_close} & {p_comp} & {p_avg} \\\\"
            latex_lines.append(line)

        latex_lines.extend(["        \\bottomrule", "    \\end{tabular}", "\\end{table}"])
        
        with open(f"{output_prefix}.tex", 'w') as f:
            f.write("\n".join(latex_lines))

        return f"Successfully generated: {output_prefix}.tex (with n sizes)"

    except Exception as e:
        return f"Error with {input_json_path}: {e}"

# --- Run Loop ---
for magic_number in [0.8, 1.0, 1.5, 3.0]:
    process_and_save_results(f"{magic_number}_lower_outliers_center_values.json", f"low_{magic_number}")
    process_and_save_results(f"{magic_number}_upper_outliers_center_values.json", f"up_{magic_number}")