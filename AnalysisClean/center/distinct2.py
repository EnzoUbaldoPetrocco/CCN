import pandas as pd
import os

def setup_distinct_directory():
    """Creates the target directory for subset data."""
    path = 'distinct'
    if not os.path.exists(path):
        os.makedirs(path)
    return path

def generate_descriptive_stats_latex(df, target_dir):
    """
    Computes mean and std deviation for each group and paradigm,
    then writes a formatted LaTeX table matching the required structure.
    """
    paradigms = ['B', 'F', 'A']
    groups = ['German', 'Italian', 'Global']
    
    tex_filepath = os.path.join(target_dir, 'table_descriptive_statistics.tex')
    
    with open(tex_filepath, 'w') as f:
        f.write(r"\begin{table}[ht]" + "\n")
        f.write(r"\centering" + "\n")
        f.write(r"\begin{tabular}{llccc}" + "\n")
        f.write(r"\toprule" + "\n")
        f.write(r"Nationality & Paradigm & Culture Closeness & Competence & Evaluation Average \\" + "\n")
        f.write(r"\midrule" + "\n")
        
        for group in groups:
            # Multirow label setup for LaTeX
            f.write(r"\multirow{3}{1.5cm}{" + group + "}\n")
            
            # Filter by group or use full dataframe for Global
            group_df = df if group == 'Global' else df[df['Nationality'] == group]
            
            for pad in paradigms:
                subset = group_df[group_df['Paradigm'] == pad]
                
                if not subset.empty:
                    # Compute Descriptive Statistics
                    mu_cc, std_cc = subset['Culture_Closeness_Avg'].mean(), subset['Culture_Closeness_Avg'].std()
                    mu_comp, std_comp = subset['Competence_Overall'].mean(), subset['Competence_Overall'].std()
                    mu_avg, std_avg = subset['Evaluation_Average'].mean(), subset['Evaluation_Average'].std()
                    
                    # Format strings with mathematical sign
                    cc_str = f"${mu_cc:.3f} \\pm {std_cc:.3f}$"
                    comp_str = f"${mu_comp:.3f} \\pm {std_comp:.3f}$"
                    avg_str = f"${mu_avg:.3f} \\pm {std_avg:.3f}$"
                else:
                    cc_str, comp_str, avg_str = "N/A", "N/A", "N/A"
                
                f.write(f" & {pad} & {cc_str} & {comp_str} & {avg_str} \\\\\n")
            
            # Add horizontal separation rule between groups (excluding the final group)
            if group != groups[-1]:
                f.write(r"\midrule" + "\n")
                
        f.write(r"\bottomrule" + "\n")
        f.write(r"\end{tabular}" + "\n")
        f.write(r"\caption{Descriptive Statistics ($\mu \pm \sigma$) by Nationality and Paradigm}" + "\n")
        f.write(r"\label{tab:nat_desc_stats_updated}" + "\n")
        f.write(r"\end{table}" + "\n")

def extract_condition_subsets_with_average(input_csv):
    """
    Partitions data into subsets and outputs a consolidated descriptive statistics LaTeX table.
    """
    df = pd.read_csv(input_csv)
    
    # Data Cleaning
    df['Nationality'] = df['Nationality'].astype(str).str.strip()
    df['Paradigm'] = df['Paradigm'].astype(str).str.strip()
    
    target_dir = setup_distinct_directory()
    
    # Calculate evaluation metrics
    metrics = ['Culture_Closeness_Avg', 'Competence_Overall']
    df['Evaluation_Average'] = df[metrics].mean(axis=1)
    
    features = ['Participant_ID', 'Culture_Closeness_Avg', 'Competence_Overall', 'Evaluation_Average']

    # 1. Extraction by Nationality and Paradigm
    for nat in df['Nationality'].unique():
        for pad in df['Paradigm'].unique():
            subset = df[(df['Nationality'] == nat) & (df['Paradigm'] == pad)]
            if not subset.empty:
                filename = f"{nat}_Paradigm_{pad}.csv"
                subset[features].to_csv(os.path.join(target_dir, filename), index=False)

    # 2. Extraction for Global Data by Paradigm
    for pad in df['Paradigm'].unique():
        global_subset = df[df['Paradigm'] == pad]
        if not global_subset.empty:
            filename = f"Global_Paradigm_{pad}.csv"
            global_subset[features].to_csv(os.path.join(target_dir, filename), index=False)

    # 3. NEW: Generate Consolidated LaTeX Statistics File
    generate_descriptive_stats_latex(df, target_dir)

if __name__ == "__main__":
    source_file = 'golden_layer/interaction_features_full.csv'
    
    if os.path.exists(source_file):
        extract_condition_subsets_with_average(source_file)
        print("Extraction complete. Subsets and 'table_descriptive_statistics.tex' generated in 'distinct/'.")
    else:
        print(f"Error: {source_file} not found.")