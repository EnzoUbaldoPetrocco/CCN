import pandas as pd
import numpy as np

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
    strong_corrs = correlated_pairs[correlated_pairs > threshold]

    return summary, correlations, strong_corrs


def take_only_one_culture(df, culture=0):
    """Filter the DataFrame to include only one culture (e.g., German)."""
    return df[df['Nationality'] == culture]

if __name__ == "__main__":
    file_path = "./Intro CCN  (Risposte).CSV"
    data = load_data(file_path)
    cleaned_data = clean_data(data)

    summary, correlations, strong_corrs = analyze_data(cleaned_data)
    
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

