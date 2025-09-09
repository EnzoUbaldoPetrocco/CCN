import pandas as pd
import numpy as np
import math

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
                correlations.abs()
                .unstack()
                .sort_values(ascending=False)
            )
            correlated_pairs = correlated_pairs[correlated_pairs < 1]  # remove self-corr
            strong_corrs = correlated_pairs[correlated_pairs > threshold]

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
    
    cleaned_data = merge_data(cleaned_data_intro, cleaned_data_center, on="Participant ID")

    print(cleaned_data.iloc[0])

    cleaned_data.to_csv("merged_cleaned_data.csv")

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