import pandas as pd
import numpy as np
import seaborn as sns
import matplotlib.pyplot as plt
from scipy.stats import pearsonr
from sklearn import preprocessing

label_map = {
         # Closeness questions
            "Which picture best describes your relationship with Italy or Germany?": "Closeness_Country",
            "Which picture best describes your relationship with Italian or German language?": "Closeness_Language",
            "Which picture best describes your relationship with Italian or German Culture?": "Closeness_Culture",

            # Personality (OCEAN 10)
            "I see myself as someone who  [... is reserved ]": "Reserved",
            "I see myself as someone who  [... is generally trusting]": "Trusting",
            "I see myself as someone who  [... tends to be lazy]": "Lazy",
            "I see myself as someone who  [... is relaxed, handles stress well]": "Relaxed",
            "I see myself as someone who  [... has few artistic interests]": "Artistic",
            "I see myself as someone who  [... is ongoing, sociable]": "Sociable",
            "I see myself as someone who  [... tends to find fault with others]": "Critical",
            "I see myself as someone who  [... does a thorough job]": "Thorough",
            "I see myself as someone who  [... get nervous easily]": "Neurotic",
            "I see myself as someone who  [... has active imagination]": "Imaginative",

            # Trust in Technology (Trust 14 items)
            "Consider expectations toward Pepper robot and express how much do you agree with the properties attributed to it (scale: 0%-100%, scroll in to see all the options).  [Function successfully]": "Function",
            "Consider expectations toward Pepper robot and express how much do you agree with the properties attributed to it (scale: 0%-100%, scroll in to see all the options).  [Act consistenly]": "Consistent",
            "Consider expectations toward Pepper robot and express how much do you agree with the properties attributed to it (scale: 0%-100%, scroll in to see all the options).  [Reliable]": "Reliable",
            "Consider expectations toward Pepper robot and express how much do you agree with the properties attributed to it (scale: 0%-100%, scroll in to see all the options).  [Predictable]": "Predictable",
            "Consider expectations toward Pepper robot and express how much do you agree with the properties attributed to it (scale: 0%-100%, scroll in to see all the options).  [Dependable]": "Dependable",
            "Consider expectations toward Pepper robot and express how much do you agree with the properties attributed to it (scale: 0%-100%, scroll in to see all the options).  [Follow directions]": "Follow_Directions",
            "Consider expectations toward Pepper robot and express how much do you agree with the properties attributed to it (scale: 0%-100%, scroll in to see all the options).  [Meet the needs of the mission]": "Mission",
            "Consider expectations toward Pepper robot and express how much do you agree with the properties attributed to it (scale: 0%-100%, scroll in to see all the options).  [Perform exactly as instructed]": "Perform",
            "Consider expectations toward Pepper robot and express how much do you agree with the properties attributed to it (scale: 0%-100%, scroll in to see all the options).  [Have errors]": "Errors",
            "Consider expectations toward Pepper robot and express how much do you agree with the properties attributed to it (scale: 0%-100%, scroll in to see all the options).  [Provide appropriate information]": "Info",
            "Consider expectations toward Pepper robot and express how much do you agree with the properties attributed to it (scale: 0%-100%, scroll in to see all the options).  [Malfunction]": "Malfunction",
            "Consider expectations toward Pepper robot and express how much do you agree with the properties attributed to it (scale: 0%-100%, scroll in to see all the options).  [Communicate with people]": "Communicate",
            "Consider expectations toward Pepper robot and express how much do you agree with the properties attributed to it (scale: 0%-100%, scroll in to see all the options).  [Provide Feedback]": "Feedback",
            "Consider expectations toward Pepper robot and express how much do you agree with the properties attributed to it (scale: 0%-100%, scroll in to see all the options).  [Unresponsive]": "Unresponsive",

            # Culture perception (first three)
            "Which picture best describes the relationship between Pepper and your country? ": "Culture_Country",
            "Which picture best describes the relationship between Pepper and your national culture? ": "Culture_National",
            "Which picture best describes the relationship between Pepper and your own preferences? ": "Culture_Preferences",

            # Robot competence / impression (Rosas scale)
            "Please rate your impression of the robot you just interacted with by selecting a point on the scale between the two adjectives. There are no right or wrong answers. ": "Capable",
            # The next 5 questions are the same text repeated in your CSV; you can map them sequentially
            # Assuming the columns appear in order for the six competence items:
            # If you have 6 columns with identical names, pandas will auto-add .1, .2, etc.
            "Please rate your impression of the robot you just interacted with by selecting a point on the scale between the two adjectives. There are no right or wrong answers. .1": "Responsive",
            "Please rate your impression of the robot you just interacted with by selecting a point on the scale between the two adjectives. There are no right or wrong answers. .2": "Interactive",
            "Please rate your impression of the robot you just interacted with by selecting a point on the scale between the two adjectives. There are no right or wrong answers. .3": "Reliable",
            "Please rate your impression of the robot you just interacted with by selecting a point on the scale between the two adjectives. There are no right or wrong answers. .4": "Competent",
            "Please rate your impression of the robot you just interacted with by selecting a point on the scale between the two adjectives. There are no right or wrong answers. .5": "Knowledgable"
    }
    

label_to_keep = [
    "Which picture best describes your relationship with Italy or Germany?",
    "Which picture best describes your relationship with Italian or German language?",
    "Which picture best describes your relationship with Italian or German Culture?",
    "mean_cultural_closeness_subj",
    "extraversion",
    "agreebleness",
    "coscientiousness",
    "neuroticism",
    "openness",
    "mean_trust_subj",
    "mean_cultural_closeness",
    "Participant ID",
    "Nationality",
    ]


def convert2latex(df, title):
    latex_code = df.to_latex(index=False, 
                         caption=title, 
                         label=f"tab:{title}",
                         column_format='lcr') # Alignment: left, center, right
    with open(f'{title}.tex', 'w') as f:
        f.write(latex_code)
    return

def load_data(file_path):
    """Load data from a CSV file into a pandas DataFrame."""
    return pd.read_csv(file_path)

def clean_data(df):
    """Clean the DataFrame by handling missing values and duplicates."""
    df = df.drop_duplicates()
    df = df.ffill().bfill()
    df = df.replace([np.inf, -np.inf], np.nan)
    df = df.replace('Disagree strongly', 1)
    df = df.replace('Disagree a little', 2)
    df = df.replace('Neither agree or disagree', 3)
    df = df.replace('Agree a little', 4)
    df = df.replace('Agree strongly', 5)
    df = df.replace('German', 0)
    df = df.replace('Italian', 1)
    def percent_to_float(x):
        if isinstance(x, str) and x.strip().endswith("%"):
            try:
                return float(x.strip().replace("%", "")) / 100
            except ValueError:
                return np.nan
        return x
    # Apply everywhere
    df = df.map(percent_to_float)
    df = df.replace(r'^\s*$', np.nan, regex=True).dropna(how="all")
    df = df.drop(df.columns[0], axis=1)
    df = df.drop(df.columns[1], axis=1)
    
    df["mean_cultural_closeness_subj"] = df.iloc[:, 2:5].mean(axis=1)
    df["mean_personality_subj"] = df.iloc[:, 5:15].mean(axis=1)
    df["extraversion"] = df.iloc[:, 10] - df.iloc[:, 5]
    df["agreebleness"] = df.iloc[:, 6] - df.iloc[:, 11]
    df["coscientiousness"] = df.iloc[:, 12] - df.iloc[:, 7]
    df["neuroticism"] = df.iloc[:, 13] - df.iloc[:, 8]
    df["openness"] = df.iloc[:, 14] - df.iloc[:, 9]
    temp = (df.iloc[:, 15:23].mean(axis=1) + df.iloc[:,24] + df.iloc[:, 26:28].mean(axis=1))/11
    temp2 = (df.iloc[:, 23] + df.iloc[:, 25] + df.iloc[:, 28])/3
    df["mean_trust_subj"] = (temp + temp2)/2
    return df

def analyze_data(df, threshold=0.8):
    """Perform basic analysis on the DataFrame and highlight correlations."""
    summary = df.describe(include='all')
    correlations = df.corr()

    # Find pairs with strong correlation
    correlated_pairs = (
        correlations
        .abs()                              # absolute correlation
        .unstack()                          # flatten into Series
        .sort_values(ascending=False)       # sort high → low
    )

    # Remove self-correlations (always 1.0 on diagonal)
    correlated_pairs = correlated_pairs[correlated_pairs < 1]

    # Keep only those above threshold
    strong_corrs = correlated_pairs[correlated_pairs.abs() > threshold]

    return summary, correlations, strong_corrs

def annotate_with_stars(r_val, p_val):
    """
    Formats the correlation coefficient (r) with significance stars based on the p-value.
    
    * p < 0.05
    ** p < 0.01
    *** p < 0.001
    """
    stars = ""
    # pd.isna handles both np.nan and pd.NA
    if pd.isna(r_val) or pd.isna(p_val):
        return ""
        
    if p_val < 0.001:
        stars = "***"
    elif p_val < 0.01:
        stars = "**"
    elif p_val < 0.05:
        stars = "*"
    
    # Format the r-value to two decimal places and append stars
    return f"{r_val:.2f}{stars}"

def generate_correlation_heatmaps(df, pt=""):
    """
    Calculates Pearson correlation coefficients and p-values for each unique 
    group defined in 'group_col', and generates a heatmap for each.
    
    Args:
        df (pd.DataFrame): The input DataFrame containing all data.
        group_col (str): The column name used for grouping (e.g., 'P').
    """
    df = df[[col for col in label_to_keep if col in df.columns]].copy()
        # 1. Subsetting and Preprocessing
        
    # Drop specified non-feature columns
    subset = df.drop(columns=["Participant ID", "Nationality"], errors="ignore")

    if subset.empty:
        return
        
    # Ensure all columns are numeric for correlation calculation
    numeric_subset = subset.select_dtypes(include=[np.number])
    if numeric_subset.shape[1] < 2:
        return

    # Use only the columns that are present in the numeric subset
    cols = numeric_subset.columns
    
    # 2. Calculate Correlation and P-value Matrices
    
    # A. Calculate R-Matrix (Correlation Coefficients) using pandas built-in corr
    # This is much faster than looping for R values.
    r_matrix = numeric_subset.corr(method='pearson')
    
    # B. Calculate P-Matrix (P-values) using scipy.stats.pearsonr
    p_matrix = pd.DataFrame(index=cols, columns=cols, dtype=float)
    
    for col1 in cols:
        for col2 in cols:
            # Correlation is symmetric, only need to calculate once per pair
            if col1 == col2:
                r_matrix.loc[col1, col2] = 1.0
                p_matrix.loc[col1, col2] = 0.0
            elif pd.isna(p_matrix.loc[col1, col2]):
                # Drop NaNs for the specific pair calculation
                valid_data = numeric_subset[[col1, col2]].dropna()
                
                if len(valid_data) >= 2:
                    try:
                        corr, p = pearsonr(valid_data[col1], valid_data[col2])
                        
                        # Store results symmetrically
                        p_matrix.loc[col1, col2] = p
                        p_matrix.loc[col2, col1] = p
                    except Exception as e:
                        # Handle cases where calculation fails (e.g., zero variance)
                        p_matrix.loc[col1, col2] = np.nan
                        p_matrix.loc[col2, col1] = np.nan
                else:
                    p_matrix.loc[col1, col2] = np.nan
                    p_matrix.loc[col2, col1] = np.nan

    # 3. Prepare Annotation Strings
    # Create a DataFrame of formatted strings (r + stars) for annotations
    annotation_df = r_matrix.copy()
    for i in cols:
        for j in cols:
            annotation_df.loc[i, j] = annotate_with_stars(r_matrix.loc[i, j], p_matrix.loc[i, j])

    print(f"R-Matrix:\n{r_matrix.round(3).head(3)}")
    print(f"Annotation Matrix:\n{annotation_df.head(3)}")

    # 4. Generate Heatmap
    
    # Mask the upper triangle
    mask = np.triu(np.ones_like(r_matrix, dtype=bool))
    r_matrix = r_matrix.rename(columns=label_map, index=label_map)
    plt.figure(figsize=(12, 10))
    sns.heatmap(
        r_matrix,            # 1. Numeric data for colors (R values)
        mask=mask,
        annot=annotation_df, # 2. String data for annotation text (R + Stars)
        fmt='s',             # IMPORTANT: Format as string ('s') to allow stars in annotation
        cmap="coolwarm",
        vmin=-1,
        vmax=1,
        linewidths=0.5,
        linecolor='white',
        cbar_kws={'label': 'Pearson Correlation Coefficient (r)'}
    )
    
    title = f"Pearson Correlation Heatmap"
    plt.title(title, fontsize=16)
    plt.tight_layout()
    
    # Save the figure
    filename = f"./{pt}Pearson_Correlation_Heatmap.png"
    plt.savefig(filename, dpi=300)
    plt.close()

    print(f"Successfully generated and saved: {filename}")


def take_only_one_culture(df, culture=0):
    """Filter the DataFrame to include only one culture (e.g., German)."""
    return df[df['Nationality'] == culture]

if __name__ == "__main__":
    file_path = "./Intro CCN  (Risposte).CSV"
    data = load_data(file_path)
    cleaned_data = clean_data(data)

    label_map = {
            # Closeness questions
            "Which picture best describes your relationship with Italy or Germany?": "Closeness_Country",
            "Which picture best describes your relationship with Italian or German language?": "Closeness_Language",
            "Which picture best describes your relationship with Italian or German Culture?": "Closeness_Culture",

            # Personality (OCEAN 10)
            "I see myself as someone who  [... is reserved ]": "Reserved",
            "I see myself as someone who  [... is generally trusting]": "Trusting",
            "I see myself as someone who  [... tends to be lazy]": "Lazy",
            "I see myself as someone who  [... is relaxed, handles stress well]": "Relaxed",
            "I see myself as someone who  [... has few artistic interests]": "Artistic",
            "I see myself as someone who  [... is ongoing, sociable]": "Sociable",
            "I see myself as someone who  [... tends to find fault with others]": "Critical",
            "I see myself as someone who  [... does a thorough job]": "Thorough",
            "I see myself as someone who  [... get nervous easily]": "Neurotic",
            "I see myself as someone who  [... has active imagination]": "Imaginative",

            # Trust in Technology (Trust 14 items)
            "Consider expectations toward Pepper robot and express how much do you agree with the properties attributed to it (scale: 0%-100%, scroll in to see all the options).  [Function successfully]": "Function",
            "Consider expectations toward Pepper robot and express how much do you agree with the properties attributed to it (scale: 0%-100%, scroll in to see all the options).  [Act consistenly]": "Consistent",
            "Consider expectations toward Pepper robot and express how much do you agree with the properties attributed to it (scale: 0%-100%, scroll in to see all the options).  [Reliable]": "Reliable",
            "Consider expectations toward Pepper robot and express how much do you agree with the properties attributed to it (scale: 0%-100%, scroll in to see all the options).  [Predictable]": "Predictable",
            "Consider expectations toward Pepper robot and express how much do you agree with the properties attributed to it (scale: 0%-100%, scroll in to see all the options).  [Dependable]": "Dependable",
            "Consider expectations toward Pepper robot and express how much do you agree with the properties attributed to it (scale: 0%-100%, scroll in to see all the options).  [Follow directions]": "Follow_Directions",
            "Consider expectations toward Pepper robot and express how much do you agree with the properties attributed to it (scale: 0%-100%, scroll in to see all the options).  [Meet the needs of the mission]": "Mission",
            "Consider expectations toward Pepper robot and express how much do you agree with the properties attributed to it (scale: 0%-100%, scroll in to see all the options).  [Perform exactly as instructed]": "Perform",
            "Consider expectations toward Pepper robot and express how much do you agree with the properties attributed to it (scale: 0%-100%, scroll in to see all the options).  [Have errors]": "Errors",
            "Consider expectations toward Pepper robot and express how much do you agree with the properties attributed to it (scale: 0%-100%, scroll in to see all the options).  [Provide appropriate information]": "Info",
            "Consider expectations toward Pepper robot and express how much do you agree with the properties attributed to it (scale: 0%-100%, scroll in to see all the options).  [Malfunction]": "Malfunction",
            "Consider expectations toward Pepper robot and express how much do you agree with the properties attributed to it (scale: 0%-100%, scroll in to see all the options).  [Communicate with people]": "Communicate",
            "Consider expectations toward Pepper robot and express how much do you agree with the properties attributed to it (scale: 0%-100%, scroll in to see all the options).  [Provide Feedback]": "Feedback",
            "Consider expectations toward Pepper robot and express how much do you agree with the properties attributed to it (scale: 0%-100%, scroll in to see all the options).  [Unresponsive]": "Unresponsive"
        }


    summary, correlations, strong_corrs = analyze_data(cleaned_data)

    corr_df_short = correlations.rename(columns=label_map, index=label_map)
    mask = np.triu(np.ones_like(corr_df_short, dtype=bool))
    plt.figure(figsize=(20, 16))
    sns.heatmap(corr_df_short, mask=mask, annot=True, fmt=".1f", cmap="coolwarm", vmin=-1, vmax=1)
    plt.title("Correlation Matrix Heatmap (Upper Triangle Hidden)")
    plt.tight_layout()
    plt.savefig("correlation_heatmap_upper.png", dpi=300)
    plt.close()
    
    #print("Data Summary:")
    #print(summary)
    summary.to_csv("data_intro_summary.csv")
    convert2latex(summary, "data_intro_summary")
    #print("\nCorrelations:")
    #print(correlations)
    correlations.to_csv("data_intro_correlations.csv")
    convert2latex(correlations, "data_intro_correlations")
    print("\nStrong Correlations (>|0.8|):")
    print(strong_corrs)

    german_data = take_only_one_culture(cleaned_data, culture=0)
    german_summary, german_correlations, german_strong_corrs = analyze_data(german_data)

    #print("\nGerman Data Summary:")
    #print(german_summary)
    german_summary.to_csv("data_intro_german_summary.csv")
    convert2latex(german_summary, "data_intro_german_summary")
    #print("\nGerman Correlations:")
    #print(german_correlations)
    german_correlations.to_csv("data_intro_german_correlations.csv")
    convert2latex(german_correlations, "data_intro_german_correlations")
    print("\nStrong Correlations in German Data (>|0.8|):")
    print(german_strong_corrs)

    corr_df_short = german_correlations.rename(columns=label_map, index=label_map)
    mask = np.triu(np.ones_like(corr_df_short, dtype=bool))
    plt.figure(figsize=(20, 16))
    sns.heatmap(corr_df_short, mask=mask, annot=True, fmt=".1f", cmap="coolwarm", vmin=-1, vmax=1)
    plt.title("Correlation Matrix Heatmap (Upper Triangle Hidden)")
    plt.tight_layout()
    plt.savefig("correlation_heatmap_german_upper.png", dpi=300)
    plt.close()



    italian_data = take_only_one_culture(cleaned_data, culture=1)
    italian_summary, italian_correlations, italian_strong_corrs = analyze_data(italian_data)
    #print("\nItalian Data Summary:")
    #print(italian_summary)
    italian_summary.to_csv("data_intro_italian_summary.csv")
    convert2latex(italian_summary, "data_intro_italian_summary")
    #print("\nItalian Correlations:")
    #print(italian_correlations)
    italian_correlations.to_csv("data_intro_italian_correlations.csv")
    convert2latex(italian_correlations, "data_intro_italian_correlations")
    print("\nStrong Correlations in Italian Data (>|0.8|):")
    print(italian_strong_corrs)

    corr_df_short = italian_correlations.rename(columns=label_map, index=label_map)
    mask = np.triu(np.ones_like(corr_df_short, dtype=bool))
    plt.figure(figsize=(20, 16))
    sns.heatmap(corr_df_short, mask=mask, annot=True, fmt=".1f", cmap="coolwarm", vmin=-1, vmax=1)
    plt.title("Correlation Matrix Heatmap (Upper Triangle Hidden)")
    plt.tight_layout()
    plt.savefig("correlation_heatmap_italian_upper.png", dpi=300)
    plt.close()


    generate_correlation_heatmaps(cleaned_data, pt="Overall_")
    generate_correlation_heatmaps(german_data, pt="German_")
    generate_correlation_heatmaps(italian_data, pt="Italian_")

