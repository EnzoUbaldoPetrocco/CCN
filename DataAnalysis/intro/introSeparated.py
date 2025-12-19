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

question_sections = {
    "culture_closeness_questions": [
        "Which picture best describes your relationship with Italy or Germany?",
        "Which picture best describes your relationship with Italian or German language?",
        "Which picture best describes your relationship with Italian or German Culture?"
    ],
    "personality_questions": [
        "I see myself as someone who  [... is reserved ]",
        "I see myself as someone who  [... is generally trusting]",
        "I see myself as someone who  [... tends to be lazy]",
        "I see myself as someone who  [... is relaxed, handles stress well]",
        "I see myself as someone who  [... has few artistic interests]",
        "I see myself as someone who  [... is ongoing, sociable]",
        "I see myself as someone who  [... tends to find fault with others]",
        "I see myself as someone who  [... does a thorough job]",
        "I see myself as someone who  [... get nervous easily]",
        "I see myself as someone who  [... has active imagination]"
    ],
    "trust_questions": [
        "Consider expectations toward Pepper robot and express how much do you agree with the properties attributed to it (scale: 0%-100%, scroll in to see all the options).  [Function successfully]",
        "Consider expectations toward Pepper robot and express how much do you agree with the properties attributed to it (scale: 0%-100%, scroll in to see all the options).  [Act consistenly]",
        "Consider expectations toward Pepper robot and express how much do you agree with the properties attributed to it (scale: 0%-100%, scroll in to see all the options).  [Reliable]",
        "Consider expectations toward Pepper robot and express how much do you agree with the properties attributed to it (scale: 0%-100%, scroll in to see all the options).  [Predictable]",
        "Consider expectations toward Pepper robot and express how much do you agree with the properties attributed to it (scale: 0%-100%, scroll in to see all the options).  [Dependable]",
        "Consider expectations toward Pepper robot and express how much do you agree with the properties attributed to it (scale: 0%-100%, scroll in to see all the options).  [Follow directions]",
        "Consider expectations toward Pepper robot and express how much do you agree with the properties attributed to it (scale: 0%-100%, scroll in to see all the options).  [Meet the needs of the mission]",
        "Consider expectations toward Pepper robot and express how much do you agree with the properties attributed to it (scale: 0%-100%, scroll in to see all the options).  [Perform exactly as instructed]",
        "Consider expectations toward Pepper robot and express how much do you agree with the properties attributed to it (scale: 0%-100%, scroll in to see all the options).  [Have errors]",
        "Consider expectations toward Pepper robot and express how much do you agree with the properties attributed to it (scale: 0%-100%, scroll in to see all the options).  [Provide appropriate information]",
        "Consider expectations toward Pepper robot and express how much do you agree with the properties attributed to it (scale: 0%-100%, scroll in to see all the options).  [Malfunction]",
        "Consider expectations toward Pepper robot and express how much do you agree with the properties attributed to it (scale: 0%-100%, scroll in to see all the options).  [Communicate with people]",
        "Consider expectations toward Pepper robot and express how much do you agree with the properties attributed to it (scale: 0%-100%, scroll in to see all the options).  [Provide Feedback]",
        "Consider expectations toward Pepper robot and express how much do you agree with the properties attributed to it (scale: 0%-100%, scroll in to see all the options).  [Unresponsive]"
    ]
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


def load_data(file_path):
    """Load data from a CSV file into a pandas DataFrame."""
    return pd.read_csv(file_path)

def clean_data(df):
    """Clean the DataFrame by handling missing values and duplicates."""
    df = df.drop_duplicates()
    df = df.ffill().bfill()
    df = df.replace([np.inf, -np.inf], np.nan)
    df = df.replace('Disagree strongly', -2)
    df = df.replace('Disagree a little', -1)
    df = df.replace('Neither agree or disagree', 0)
    df = df.replace('Agree a little', 1)
    df = df.replace('Agree strongly', 2)
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
    #df = df[[col for col in label_to_keep if col in df.columns]].copy()
    df.rename(columns=label_map, inplace=True)
        # 1. Subsetting and Preprocessing
        
    # Drop specified non-feature columns
    subset = df.drop(columns=["Participant ID"], errors="ignore")

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

    #print(f"R-Matrix:\n{r_matrix.round(3).head(3)}")
    #print(f"Annotation Matrix:\n{annotation_df.head(3)}")

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

    plt.figure(figsize=(12, 10))
    sns.heatmap(
        p_matrix,            # 1. Numeric data for colors (P values)
        mask=mask,
        annot=True,         # 2. Annotate with P values
        fmt=".3f",          # Format P values to three decimal places
        cmap="YlGnBu_r",    # Reverse colormap for better visibility
        vmin=0,
        vmax=0.05,
        linewidths=0.5,
        linecolor='white',
        cbar_kws={'label': 'P-Value'}
    )
    title = f"P-Value Heatmap"
    plt.title(title, fontsize=16)
    plt.tight_layout()
    # Save the figure
    filename = f"./{pt}P_Value_Heatmap.png"
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
    german_data = take_only_one_culture(cleaned_data, culture=0)
    italian_data = take_only_one_culture(cleaned_data, culture=1)

    #generate_correlation_heatmaps(cleaned_data, pt="Overall_")
    #generate_correlation_heatmaps(german_data, pt="German_")
    #generate_correlation_heatmaps(italian_data, pt="Italian_")

    print("Data Cleaned columns")
    print(cleaned_data.columns)

    questions_culturevspersonality = question_sections["culture_closeness_questions"] + question_sections["personality_questions"]
    clean_data_culturevspersonality = cleaned_data[questions_culturevspersonality].copy()
    german_data_culturevspersonality = german_data[questions_culturevspersonality].copy()
    italian_data_culturevspersonality = italian_data[questions_culturevspersonality].copy()
    generate_correlation_heatmaps(german_data_culturevspersonality, pt="German_CultureVsPersonality_")
    generate_correlation_heatmaps(italian_data_culturevspersonality, pt="Italian_CultureVsPersonality_")
    generate_correlation_heatmaps(clean_data_culturevspersonality, pt="CultureVsPersonality_")

    questions_culturevstrust = question_sections["culture_closeness_questions"] + question_sections["trust_questions"]
    clean_data_culturevstrust = cleaned_data[questions_culturevstrust].copy()
    german_data_culturevstrust = german_data[questions_culturevstrust].copy()
    italian_data_culturevstrust = italian_data[questions_culturevstrust].copy()
    generate_correlation_heatmaps(german_data_culturevstrust, pt="German_CultureVsTrust_")
    generate_correlation_heatmaps(italian_data_culturevstrust, pt="Italian_CultureVsTrust_")
    generate_correlation_heatmaps(clean_data_culturevstrust, pt="CultureVsTrust_")

    questions_personalityvstrust = question_sections["personality_questions"] + question_sections["trust_questions"]
    clean_data_personalityvstrust = cleaned_data[questions_personalityvstrust].copy()
    german_data_personalityvstrust = german_data[questions_personalityvstrust].copy()
    italian_data_personalityvstrust = italian_data[questions_personalityvstrust].copy()
    generate_correlation_heatmaps(german_data_personalityvstrust, pt="German_PersonalityVsTrust_")
    generate_correlation_heatmaps(italian_data_personalityvstrust, pt="Italian_PersonalityVsTrust_")
    generate_correlation_heatmaps(clean_data_personalityvstrust, pt="PersonalityVsTrust_")

    