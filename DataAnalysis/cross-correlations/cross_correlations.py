import pandas as pd
import numpy as np
import math
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
    "mean_competence",
    "Participant ID",
    "P",
    "Nationality",
    "Delta_Type"
    ]

def load_data(file_path):
    """Load data from a CSV file into a pandas DataFrame."""
    return pd.read_csv(file_path)

def merge_data(df1, df2, on="Participant ID"):
    """Merge two DataFrames on a specified column."""
    df = pd.merge(df1, df2, on=on)
    df = df.drop(columns=["Nationality_y"], errors="ignore")
    df = df.rename(columns={"Nationality_x": "Nationality"})
    return df

def check_user_validity(
    df, 
    user_id_col="Participant ID", 
    group_col="P", 
    nationality_col="Nationality", 
    required_groups={"A", "B", "F"}
):
    """Check which users have all required groups (A,B,F) and consistent nationality."""
    
    user_groups = df.groupby(user_id_col)[group_col].apply(set)
    user_nationality = df.groupby(user_id_col)[nationality_col].nunique()
    
    # Valid if: has all groups + only 1 nationality
    valid_users = user_groups[
        user_groups.apply(lambda g: required_groups.issubset(g))
    ].index
    
    valid_users = [u for u in valid_users if user_nationality.loc[u] == 1]
    
    # Invalid users (missing groups or inconsistent nationality)
    invalid_users = set(df[user_id_col].unique()) - set(valid_users)
    
    return valid_users, invalid_users

def clean_data_intro(df):
    """Clean the DataFrame by handling missing values and duplicates."""
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
    #df = df.drop(df.columns[1], axis=1)
    
    df["mean_cultural_closeness_subj"] = df.iloc[:, 2:5].mean(axis=1)
    #df["mean_personality_subj"] = df.iloc[:, 5:15].mean(axis=1)
    df["extraversion"] = df.iloc[:, 10] - df.iloc[:, 5]
    df["agreebleness"] = df.iloc[:, 6] - df.iloc[:, 11]
    df["coscientiousness"] = df.iloc[:, 12] - df.iloc[:, 7]
    df["neuroticism"] = df.iloc[:, 13] - df.iloc[:, 8]
    df["openness"] = df.iloc[:, 14] - df.iloc[:, 9]
    temp = (df.iloc[:, 15:23].mean(axis=1) + df.iloc[:,24] + df.iloc[:, 26:28].mean(axis=1))/11
    temp2 = (df.iloc[:, 23] + df.iloc[:, 25] + df.iloc[:, 28])/3
    df["mean_trust_subj"] = (temp + temp2)/2

    for column in df.columns:
        if column != "Participant ID" and column != "Nationality":
            scaler = preprocessing.StandardScaler()
            if pd.api.types.is_numeric_dtype(df[column]):
                df[column] = scaler.fit_transform(df[[column]])
    return df

def clean_data_center(df):
    """Clean the DataFrame by handling missing values and duplicates."""
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
    # Mean of first 3 values per row
    df["mean_cultural_closeness"] = df.iloc[:, -9:-6].mean(axis=1)
    # Mean of last 6 values per row
    df["mean_competence"] = df.iloc[:, -6:].mean(axis=1)
    #df = df.drop(df.columns[1], axis=1)
    for column in df.columns:
        if column != "Participant ID" and column != "Nationality":
            scaler = preprocessing.StandardScaler()
            if pd.api.types.is_numeric_dtype(df[column]):
                df[column] = scaler.fit_transform(df[[column]])
    return df

def analyze_data_with_nationality(df, threshold=0.8, user_id_col="Participant ID", group_col="P", nationality_col="Nationality"):
    """Perform correlations by P and nationality, excluding user ID."""
    results = {}

    # Iterate over combinations of P and nationality
    for p_value in df[group_col].unique():
        for n_value in df[nationality_col].unique():
            subset = df[(df[group_col] == p_value) & (df[nationality_col] == n_value)]

            if subset.empty:
                continue

            # Drop non-feature columns
            subset = subset.drop(columns=[user_id_col, group_col, nationality_col], errors="ignore")

            subset_summary = subset.describe(include='all')

            subset.to_csv(f"subset_group{p_value}_nat{n_value}.csv")

            # Correlation matrix
            correlations = subset.corr()

            # Strong correlations
            correlated_pairs = (
                correlations
                .unstack()
                .sort_values(ascending=False)
            )
            correlated_pairs = correlated_pairs[correlated_pairs.abs() < 1]  # remove self-corr
            strong_corrs = correlated_pairs[correlated_pairs.abs() > threshold]

            # Save results
            results[(p_value, n_value)] = {
                "correlations": correlations,
                "strong_corrs": strong_corrs,
                "summary": subset_summary
            }

    return results

def analyze_data_by_group(df, threshold=0.8, user_id_col="Participant ID", group_col="P"):
    """Perform correlations by P only, excluding user ID."""
    results = {}

    # Iterate over groups (A, B, F, etc.)
    for p_value in df[group_col].unique():
        subset = df[df[group_col] == p_value]

        if subset.empty:
            continue

        # Drop non-feature columns
        subset = subset.drop(columns=[user_id_col, group_col], errors="ignore")

        # Summary stats
        subset_summary = subset.describe(include='all')

        subset.to_csv(f"subset_group{p_value}.csv")

        # Correlation matrix
        correlations = subset.corr()

        # Strong correlations
        correlated_pairs = (
            correlations
            .unstack()
            .sort_values(ascending=False)
        )
        correlated_pairs = correlated_pairs[correlated_pairs.abs() < 1]  # remove self-corr
        strong_corrs = correlated_pairs[correlated_pairs > threshold]

        

        # Save results
        results[p_value] = {
            "correlations": correlations,
            "strong_corrs": strong_corrs,
            "summary": subset_summary
        }
        # Summary stats
        subset_summary = subset.describe(include='all')

        subset.to_csv(f"subset_group{p_value}.csv")

        # Correlation matrix
        correlations = subset.corr()

        # Strong correlations
        correlated_pairs = (
            correlations.abs()
            .unstack()
            .sort_values(ascending=False)
        )
        correlated_pairs = correlated_pairs[correlated_pairs < 1]  # remove self-corr
        strong_corrs = correlated_pairs[correlated_pairs > threshold]

        # Save results
        results[p_value] = {
            "correlations": correlations,
            "strong_corrs": strong_corrs,
            "summary": subset_summary
        }

    return results

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

def generate_grouped_correlation_heatmaps(df, group_col="P", pt=""):
    """
    Calculates Pearson correlation coefficients and p-values for each unique 
    group defined in 'group_col', and generates a heatmap for each.
    
    Args:
        df (pd.DataFrame): The input DataFrame containing all data.
        group_col (str): The column name used for grouping (e.g., 'P').
    """
    df = df[[col for col in label_to_keep if col in df.columns]].copy()
    for p_value in df[group_col].unique():
        # 1. Subsetting and Preprocessing
        print(f"\n--- Processing Group: {p_value} ---")
        subset = df[df[group_col] == p_value].copy()
        
        # Drop specified non-feature columns
        subset = subset.drop(columns=["Participant ID", group_col, "Nationality"], errors="ignore")

        if subset.empty:
            print(f"Skipping group {p_value}: No data remaining.")
            continue
            
        # Ensure all columns are numeric for correlation calculation
        numeric_subset = subset.select_dtypes(include=[np.number])
        if numeric_subset.shape[1] < 2:
            print(f"Skipping group {p_value}: Less than two numeric columns remaining.")
            continue

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
        
        title = f"Pearson Correlation Heatmap for Group: {p_value}"
        plt.title(title, fontsize=16)
        plt.tight_layout()
        
        # Save the figure
        filename = f"./{pt}Pearson_Correlation_Heatmap_{p_value}.png"
        plt.savefig(filename, dpi=300)
        plt.close()

        print(f"Successfully generated and saved: {filename}")

def take_only_one_culture(df, culture=0):
    """Filter the DataFrame to include only one culture (e.g., German)."""
    return df[df['Nationality'] == culture]

     

if __name__ == "__main__":
    file_path_center = "../center/Center CCN (Risposte).csv"
    file_path_intro = "../intro/Intro CCN  (Risposte).CSV"

    

    
    data_intro = load_data(file_path_intro)
    data_center = load_data(file_path_center)
    valid_users, invalid_users = check_user_validity(data_center)
    print("✅ Valid Users:", valid_users)
    print("❌ Invalid Users:", invalid_users)
    # Filter only valid users
    df_valid = data_center[data_center["Participant ID"].isin(valid_users)]
    cleaned_data_center = clean_data_center(df_valid)
    cleaned_data_intro = clean_data_intro(data_intro)
    center_copy = cleaned_data_center.copy()
    intro_copy = cleaned_data_intro.copy()
    
    cleaned_data = merge_data(cleaned_data_intro, cleaned_data_center, on="Participant ID")

    print(cleaned_data.iloc[0])

    cleaned_data.to_csv("all.csv")
    
    results = analyze_data_with_nationality(cleaned_data, threshold=0.8, user_id_col="Participant ID", group_col="P", nationality_col="Nationality")

    for (group, nationality), data in results.items():
        print("=" * 60)
        print(f"Group {group} | Nationality {nationality}")

        #print("\nCorrelation Matrix:")
        #print(data["correlations"])

        print("\nHighly Correlated Pairs (>|0.8|):")
        if data["strong_corrs"].empty:
            print("None found.")
        else:
            print(data["strong_corrs"])
        # Optionally save to CSV
        data["correlations"].to_csv(f"group{group}_nat{nationality}_correlations.csv")  
        data["strong_corrs"].to_csv(f"group{group}_nat{nationality}_strong_corrs.csv")
        data["summary"].to_csv(f"group{group}_nat{nationality}_summary.csv")

        if data["correlations"].empty:
            continue
        corr_df_short = data["correlations"].rename(columns=label_map, index=label_map)
        mask = np.triu(np.ones_like(corr_df_short, dtype=bool))
        plt.figure(figsize=(20, 16))
        sns.heatmap(corr_df_short, mask=mask, annot=True, fmt=".1f", cmap="coolwarm", vmin=-1, vmax=1)
        plt.title("Correlation Matrix Heatmap (Upper Triangle Hidden)")
        plt.tight_layout()
        plt.savefig(f"./data_center_group{group}_nat{nationality}_correlation_heatmap_upper.png", dpi=300)
        plt.close()

    results = analyze_data_by_group(cleaned_data, threshold=0.8, user_id_col="Participant ID", group_col="P")

    for group, data in results.items():
        print("=" * 60)
        print(f"Group {group}")

        #print("\nCorrelation Matrix:")
        #print(data["correlations"])

        print("\nHighly Correlated Pairs (>|0.8|):")
        if data["strong_corrs"].empty:
            print("None found.")
        else:
            print(data["strong_corrs"])
        # Optionally save to CSV
        data["correlations"].to_csv(f"group{group}_correlations.csv")  
        data["strong_corrs"].to_csv(f"group{group}_strong_corrs.csv")
        data["summary"].to_csv(f"group{group}_summary.csv")

        if data["correlations"].empty:
            continue
        corr_df_short = data["correlations"].rename(columns=label_map, index=label_map)
        mask = np.triu(np.ones_like(corr_df_short, dtype=bool))
        plt.figure(figsize=(20, 16))
        sns.heatmap(corr_df_short, mask=mask, annot=True, fmt=".1f", cmap="coolwarm", vmin=-1, vmax=1)
        plt.title("Correlation Matrix Heatmap (Upper Triangle Hidden)")
        plt.tight_layout()
        plt.savefig(f"./data_center_group{group}_correlation_heatmap_upper.png", dpi=300)
        plt.close()

    print("--- Data Snapshot Before Processing ---")
    print(cleaned_data.head())
    print("-" * 50)

    # Run the analysis function
    generate_grouped_correlation_heatmaps(cleaned_data, group_col="P")

    generate_grouped_correlation_heatmaps(cleaned_data, group_col="Nationality")

    for nat in [0, 1]:
        deltas_nat = take_only_one_culture(cleaned_data, culture=nat)
        path_prefix = "DE" if nat == 0 else "IT"
        print("Path prefix: ", path_prefix)
        generate_grouped_correlation_heatmaps(deltas_nat, group_col="P", pt=path_prefix)



    ########################################################################
    ######################## DELTAS ANALYSIS  ################################
    ########################################################################

    # Calculate deltas for each user between P=A and P=F and P=B and P=F
    deltas_AF = []
    deltas_AB = []
    deltas_FB = []
    for user in valid_users:
        print(f"Processing deltas for user: {user}")
        user_data = center_copy[center_copy["Participant ID"] == user]
        data_A = user_data[user_data["P"] == "A"]
        data_B = user_data[user_data["P"] == "B"]
        data_F = user_data[user_data["P"] == "F"]

        if not data_A.empty and not data_B.empty:
            delta_AB = data_A.iloc[0].drop(["Informazioni cronologiche", "Nationality", "Participant ID", "P"], errors="ignore") - data_B.iloc[0].drop(["Informazioni cronologiche","Nationality", "Participant ID", "P"], errors="ignore")
            with pd.option_context('display.max_rows', None, 'display.max_columns', None):  # more options can be specified also
                print(f"Delta AF for user {user}:\n{delta_AB}")
            delta_AB["Participant ID"] = user
            delta_AB["Delta_Type"] = "A-B"
            deltas_AB.append(delta_AB)

        if not data_F.empty and not data_B.empty:
            delta_FB = data_F.iloc[0].drop(["Informazioni cronologiche","Nationality", "Participant ID", "P"], errors="ignore") - data_B.iloc[0].drop(["Informazioni cronologiche","Nationality", "Participant ID", "P"], errors="ignore")
            delta_FB["Participant ID"] = user
            delta_FB["Delta_Type"] = "F-B"
            deltas_FB.append(delta_FB)

        if not data_A.empty and not data_F.empty:
            delta_AF = data_A.iloc[0].drop(["Informazioni cronologiche","Nationality", "Participant ID", "P"], errors="ignore") - data_F.iloc[0].drop(["Informazioni cronologiche","Nationality", "Participant ID", "P"], errors="ignore")
            delta_AF["Participant ID"] = user
            delta_AF["Delta_Type"] = "A-F"
            deltas_AF.append(delta_AF)

        
    deltas_df_AB_center = pd.DataFrame(deltas_AB)
    deltas_df_FB_center = pd.DataFrame(deltas_FB)
    deltas_df_AF_center = pd.DataFrame(deltas_AF)
    deltas_df_AB = merge_data(deltas_df_AB_center, intro_copy, on="Participant ID")
    deltas_df_FB = merge_data(deltas_df_FB_center, intro_copy, on="Participant ID")
    deltas_df_AF = merge_data(deltas_df_AF_center, intro_copy, on="Participant ID")

    #deltas = merge_data(deltas_df_AB, deltas_df_FB, on="Participant ID")

    generate_grouped_correlation_heatmaps(deltas_df_AB, group_col="Delta_Type", pt="AB/")
    generate_grouped_correlation_heatmaps(deltas_df_FB, group_col="Delta_Type", pt="FB/")
    generate_grouped_correlation_heatmaps(deltas_df_AF, group_col="Delta_Type", pt="AF/")
    # Run the analysis function on deltas

    for nat in [0, 1]:
        deltas_nat_FB = take_only_one_culture(deltas_df_FB, culture=nat)
        path_prefix = "FB/DE/" if nat == 0 else "FB/IT/"
        print("Path prefix: ", path_prefix)
        generate_grouped_correlation_heatmaps(deltas_nat_FB, group_col="Delta_Type", pt=path_prefix)

    for nat in [0, 1]:
        deltas_nat_AF = take_only_one_culture(deltas_df_AF, culture=nat)
        path_prefix = "AF/DE/" if nat == 0 else "AF/IT/"
        generate_grouped_correlation_heatmaps(deltas_nat_AF, group_col="Delta_Type", pt=path_prefix)

    for nat in [0, 1]:
        deltas_nat_AB = take_only_one_culture(deltas_df_AB, culture=nat)
        path_prefix = "AB/DE/" if nat == 0 else "AB/IT/"
        generate_grouped_correlation_heatmaps(deltas_nat_AB, group_col="Delta_Type", pt=path_prefix)

    
    
