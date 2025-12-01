import pandas as pd
import numpy as np
import os
import seaborn as sns
import matplotlib.pyplot as plt

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
    #df = df.drop(df.columns[1], axis=1)
    
    # As I removed the Participant ID column above, the indices have shifted by 1
    df["mean_cultural_closeness_subj"] = df.iloc[:, 3:6].mean(axis=1)
    df["mean_personality_subj"] = df.iloc[:, 6:16].mean(axis=1)
    df["extraversion"] = df.iloc[:, 11] - df.iloc[:, 6]
    df["agreebleness"] = df.iloc[:, 7] - df.iloc[:, 12]
    df["coscientiousness"] = df.iloc[:, 13] - df.iloc[:, 8]
    df["neuroticism"] = df.iloc[:, 14] - df.iloc[:, 9]
    df["openness"] = df.iloc[:, 15] - df.iloc[:, 10]
    temp = (df.iloc[:, 16:24].mean(axis=1) + df.iloc[:,25] + df.iloc[:, 27:29].mean(axis=1))/11
    temp2 = (df.iloc[:, 24] + df.iloc[:, 26] + df.iloc[:, 29])/3
    df["mean_trust_subj"] = (temp + temp2)/2
    return df

def remove_top_percentage(df, columns, percentage=0.10, use_mean=False):
    """
    Remove the top percentage of rows based on one column or the mean of several columns.

    Parameters:
    - df (pd.DataFrame): The dataset.
    - columns (str or list): Column name or list of columns to consider.
    - percentage (float): Fraction of rows to remove (0.10 = remove top 10%).
    - use_mean (bool): If True, compute mean across `columns` and rank by that mean.

    Returns:
    - df_filtered (pd.DataFrame): DataFrame after removing rows.
    - removed_rows (pd.DataFrame): The removed top-percentage rows.
    """

    # Make sure columns is a list
    if isinstance(columns, str):
        columns = [columns]

    if use_mean:
        scores = df[columns].mean(axis=1)
    else:
        if len(columns) != 1:
            raise ValueError("When use_mean=False, provide exactly one column name.")
        scores = df[columns[0]]

    # Number of rows to remove
    n_remove = int(len(df) * percentage)

    # Get indices of rows with highest scores
    top_idx = scores.sort_values(ascending=False).head(n_remove).index

    # Split into kept + removed datasets
    removed_rows = df.loc[top_idx]
    df_filtered = df.drop(top_idx)

    return df_filtered, removed_rows


def analyze_data(df, threshold=0.8):
    """Perform basic analysis on the DataFrame and highlight correlations."""
    summary = df.describe(include='all')
    without_ID = df.select_dtypes(include=[np.number])
    correlations = without_ID.corr()

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


def take_only_one_culture(df, culture=0):
    """Filter the DataFrame to include only one culture (e.g., German)."""
    return df[df['Nationality'] == culture]

import re

def sanitize_filename(name):
    # Remove characters not allowed in Windows filenames
    return re.sub(r'[<>:"/\\|?*]', '', name)


if __name__ == "__main__":
    file_path = "../Intro CCN  (Risposte).CSV"
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


    for percentage in [0.5, 0.7, 0.9]:
        for sentence in [
            "Which picture best describes your relationship with Italy or Germany?",
            "Which picture best describes your relationship with Italian or German language?",
            "Which picture best describes your relationship with Italian or German Culture?"
        ]:
            os.makedirs(f"./{percentage}", exist_ok=True)
            safe_sentence = sanitize_filename(sentence).replace(" ", "_")
            base_path = f"./{percentage}/{safe_sentence}/"
            os.makedirs(base_path, exist_ok=True)
            
            filtered_df, removed = remove_top_percentage(
                cleaned_data,
                columns=sentence,
                percentage=percentage
                )
            
            output_removed_path = f"./{base_path}/data_intro.csv"
            removed.to_csv(output_removed_path)
            print(f"Removed top {int(percentage*100)}% rows saved to {output_removed_path}")

            remaining_ids = filtered_df["Participant ID"]
            output_remaining_ids_path = f"{base_path}/remaining_participant_ids.csv"
            remaining_ids.to_csv(output_remaining_ids_path, index=False)
            print(f"Remaining participant IDs saved to {output_remaining_ids_path}")

            summary, correlations, strong_corrs = analyze_data(filtered_df)
            output_filtered_summary = f"./{base_path}/data_intro_filtered_summary.csv"
            summary.to_csv(output_filtered_summary)
        
            output_filtered_correlations = f"./{base_path}/data_intro_filtered_correlations.csv"
            correlations.to_csv(output_filtered_correlations)
            print(f"Filtered data summary saved to {output_filtered_summary}")
            print(f"Filtered data correlations saved to {output_filtered_correlations}")

            corr_df_short = correlations.rename(columns=label_map, index=label_map)
            mask = np.triu(np.ones_like(corr_df_short, dtype=bool))
            plt.figure(figsize=(20, 16))
            sns.heatmap(corr_df_short, mask=mask, annot=True, fmt=".1f", cmap="coolwarm", vmin=-1, vmax=1)
            plt.title("Correlation Matrix Heatmap (Upper Triangle Hidden)")
            plt.tight_layout()
            plt.savefig(f"./{base_path}/correlation_heatmap_upper.png", dpi=300)
            plt.close()

            german_data = take_only_one_culture(filtered_df, culture=0)
            german_summary, german_correlations, german_strong_corrs = analyze_data(german_data)
            output_german_summary = f"./{base_path}/data_intro_german_summary.csv"
            german_summary.to_csv(output_german_summary)
            output_german_correlations = f"./{base_path}/data_intro_german_correlations.csv"
            german_correlations.to_csv(output_german_correlations)
            print(f"German data summary saved to {output_german_summary}")
            print(f"German data correlations saved to {output_german_correlations}")

            corr_df_short = german_correlations.rename(columns=label_map, index=label_map)
            mask = np.triu(np.ones_like(corr_df_short, dtype=bool))
            plt.figure(figsize=(20, 16))
            sns.heatmap(corr_df_short, mask=mask, annot=True, fmt=".1f", cmap="coolwarm", vmin=-1, vmax=1)
            plt.title("Correlation Matrix Heatmap (Upper Triangle Hidden)")
            plt.tight_layout()
            plt.savefig(f"./{base_path}/correlation_heatmap_german_upper.png", dpi=300)
            plt.close()


            italian_data = take_only_one_culture(filtered_df, culture=1)
            italian_summary, italian_correlations, italian_strong_corrs = analyze_data(italian_data)
            output_italian_summary = f"./{base_path}/data_intro_italian_summary.csv"
            italian_summary.to_csv(output_italian_summary)
            output_italian_correlations = f"./{base_path}/data_intro_italian_correlations.csv"
            italian_correlations.to_csv(output_italian_correlations)
            print(f"Italian data summary saved to {output_italian_summary}")
            print(f"Italian data correlations saved to {output_italian_correlations}")

            corr_df_short = italian_correlations.rename(columns=label_map, index=label_map)
            mask = np.triu(np.ones_like(corr_df_short, dtype=bool))
            plt.figure(figsize=(20, 16))
            sns.heatmap(corr_df_short, mask=mask, annot=True, fmt=".1f", cmap="coolwarm", vmin=-1, vmax=1)
            plt.title("Correlation Matrix Heatmap (Upper Triangle Hidden)")
            plt.tight_layout()
            plt.savefig(f"./{base_path}/correlation_heatmap_italian_upper.png", dpi=300)
            plt.close()

        os.makedirs(f"./{percentage}", exist_ok=True)
        os.makedirs(f"./{percentage}/mean/", exist_ok=True)
        base_path = f"./{percentage}/mean/"
        os.makedirs(base_path, exist_ok=True)
    
        filtered_df, removed = remove_top_percentage(
            cleaned_data,
            columns=[
                "Which picture best describes your relationship with Italy or Germany?",
                "Which picture best describes your relationship with Italian or German language?",
                "Which picture best describes your relationship with Italian or German Culture?"
            ],
            percentage=percentage,
            use_mean=True
            )
        output_removed_path = f"./{base_path}/data_intro.csv"
        removed.to_csv(output_removed_path)
        print(f"Removed top {int(percentage*100)}% rows saved to {output_removed_path}")

        remaining_ids = filtered_df["Participant ID"]
        remaining_ids.to_csv(f"{base_path}/remaining_participant_ids.csv", index=False)


        summary, correlations, strong_corrs = analyze_data(filtered_df)
        output_filtered_summary = f"./{base_path}/data_intro_filtered_summary.csv"
        summary.to_csv(output_filtered_summary)
        output_filtered_correlations = f"./{base_path}/data_intro_filtered_correlations.csv"
        correlations.to_csv(output_filtered_correlations)
        print(f"Filtered data summary saved to {output_filtered_summary}")
        print(f"Filtered data correlations saved to {output_filtered_correlations}")

        corr_df_short = correlations.rename(columns=label_map, index=label_map)
        mask = np.triu(np.ones_like(corr_df_short, dtype=bool))
        plt.figure(figsize=(20, 16))
        sns.heatmap(corr_df_short, mask=mask, annot=True, fmt=".1f", cmap="coolwarm", vmin=-1, vmax=1)
        plt.title("Correlation Matrix Heatmap (Upper Triangle Hidden)")
        plt.tight_layout()
        plt.savefig(f"./{base_path}/correlation_heatmap_upper.png", dpi=300)
        plt.close()

        german_data = take_only_one_culture(filtered_df, culture=0)
        german_summary, german_correlations, german_strong_corrs = analyze_data(german_data)
        output_german_summary = f"./{base_path}/data_intro_german_summary.csv"
        german_summary.to_csv(output_german_summary)
        output_german_correlations = f"./{base_path}/data_intro_german_correlations.csv"
        german_correlations.to_csv(output_german_correlations)
        print(f"German data summary saved to {output_german_summary}")
        print(f"German data correlations saved to {output_german_correlations}")

        corr_df_short = german_correlations.rename(columns=label_map, index=label_map)
        mask = np.triu(np.ones_like(corr_df_short, dtype=bool))
        plt.figure(figsize=(20, 16))
        sns.heatmap(corr_df_short, mask=mask, annot=True, fmt=".1f", cmap="coolwarm", vmin=-1, vmax=1)
        plt.title("Correlation Matrix Heatmap (Upper Triangle Hidden)")
        plt.tight_layout()
        plt.savefig(f"./{base_path}/correlation_heatmap_german_upper.png", dpi=300)
        plt.close()

        italian_data = take_only_one_culture(filtered_df, culture=1)
        italian_summary, italian_correlations, italian_strong_corrs = analyze_data(italian_data)
        output_italian_summary = f"./{base_path}/data_intro_italian_summary.csv"
        italian_summary.to_csv(output_italian_summary)
        output_italian_correlations = f"./{base_path}/data_intro_italian_correlations.csv"
        italian_correlations.to_csv(output_italian_correlations)
        print(f"Italian data summary saved to {output_italian_summary}")
        print(f"Italian data correlations saved to {output_italian_correlations}")

        corr_df_short = italian_correlations.rename(columns=label_map, index=label_map)
        mask = np.triu(np.ones_like(corr_df_short, dtype=bool))
        plt.figure(figsize=(20, 16))
        sns.heatmap(corr_df_short, mask=mask, annot=True, fmt=".1f", cmap="coolwarm", vmin=-1, vmax=1)
        plt.title("Correlation Matrix Heatmap (Upper Triangle Hidden)")
        plt.tight_layout()
        plt.savefig(f"./{base_path}/correlation_heatmap_italian_upper.png", dpi=300)
        plt.close()


