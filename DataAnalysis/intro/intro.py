import pandas as pd
import numpy as np
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
    #print("\nCorrelations:")
    #print(correlations)
    correlations.to_csv("data_intro_correlations.csv")
    print("\nStrong Correlations (>|0.8|):")
    print(strong_corrs)

    german_data = take_only_one_culture(cleaned_data, culture=0)
    german_summary, german_correlations, german_strong_corrs = analyze_data(german_data)

    #print("\nGerman Data Summary:")
    #print(german_summary)
    german_summary.to_csv("data_intro_german_summary.csv")
    #print("\nGerman Correlations:")
    #print(german_correlations)
    german_correlations.to_csv("data_intro_german_correlations.csv")
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
    #print("\nItalian Correlations:")
    #print(italian_correlations)
    italian_correlations.to_csv("data_intro_italian_correlations.csv")
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

