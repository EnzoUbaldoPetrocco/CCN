import pandas as pd
import numpy as np
import math
import seaborn as sns
from scipy import stats
import matplotlib.pyplot as plt
import pingouin as pg
from itertools import combinations
import os

def load_data(file_path):
    """Load data from a CSV file into a pandas DataFrame."""
    return pd.read_csv(file_path)

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

            # Correlation matrix
            correlations = subset.corr()

            # Strong correlations
            correlated_pairs = (
                correlations
                .unstack()
                .sort_values(ascending=False)
            )
            correlated_pairs = correlated_pairs[correlated_pairs < 1]  # remove self-corr
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
    file_path = "../Center CCN (Risposte).csv"
    data = load_data(file_path)
    valid_users, invalid_users = check_user_validity(data)

    cols = ["Which picture best describes the relationship between Pepper and your country? ",
            "Which picture best describes the relationship between Pepper and your national culture? ",
            "Which picture best describes the relationship between Pepper and your own preferences? ",
            "Please rate your impression of the robot you just interacted with by selecting a point on the scale between the two adjectives. There are no right or wrong answers. ",
            "Please rate your impression of the robot you just interacted with by selecting a point on the scale between the two adjectives. There are no right or wrong answers. ",
            "Please rate your impression of the robot you just interacted with by selecting a point on the scale between the two adjectives. There are no right or wrong answers. ",
            "Please rate your impression of the robot you just interacted with by selecting a point on the scale between the two adjectives. There are no right or wrong answers. ",
            "Please rate your impression of the robot you just interacted with by selecting a point on the scale between the two adjectives. There are no right or wrong answers. ",
            "Please rate your impression of the robot you just interacted with by selecting a point on the scale between the two adjectives. There are no right or wrong answers. "
            ]

    def check_normality(data):
        if len(data) < 3:
            return False
        stat, p = stats.shapiro(data)
        print(f"Shapiro-Wilk for Group: p-value = {p}")
        return p > 0.05  # Null hypothesis: data is normally distributed
    
    def homogeneity_of_variances(data1, data2):
        """Check homogeneity of variances using Levene's test."""
        stat, p = stats.levene(data1, data2)
        print(f"Levene's test: p-value = {p}")
        return p > 0.05  # Null hypothesis: equal variances
    
    def perform_ttest_by_culture(df, column="mean_cultural_closeness", group_col="P"):
        """Perform t-tests between groups for a specific column."""
        p_values = df[group_col].unique()
        
        print(f"T-test results for '{column}' by {group_col} values:\n")
        for i, p1 in enumerate(p_values):
            for p2 in p_values[i+1:]:
                group1 = df[df[group_col] == p1][column].dropna()
                group2 = df[df[group_col] == p2][column].dropna()
                if len(group1) > 0 and len(group2) > 0:
                    if (check_normality(group1) and check_normality(group2)) and homogeneity_of_variances(group1, group2):
                        t_stat, p_value = stats.ttest_ind(group1, group2, equal_var=True)
                        print(f"{group_col}={p1} vs {group_col}={p2}:")
                        print(f"  t-statistic: {t_stat:.4f}, p-value: {p_value:.4f}\n")
                    else:
                        print(f"Not normal distribution or unequal variances between {group_col}={p1} and {group_col}={p2}.")
                        # Use Welch's T-test (do not assume equal variances)
                        ttest_result = stats.ttest_ind(group1, group2, equal_var=False)
                        print(f"{group_col}={p1} vs {group_col}={p2}:")
                        print(f"  Welch's t-statistic: {ttest_result.statistic:.4f}, p-value: {ttest_result.pvalue:.4f}\n")

    def perform_f_oneway(df, column="mean_cultural_closeness", group_col="P"):
        """Perform one-way ANOVA between groups for a specific column."""
        groups = [df[df[group_col] == g][column].dropna() for g in df[group_col].unique()]
        f_stat, p_value = stats.f_oneway(*groups)
        print(f"One-way ANOVA for '{column}' by {group_col}: F-statistic = {f_stat}, p-value = {p_value}\n")
        # H0: The medians (ranks) of all groups are equal.
        h_statistic, p_kruskal = stats.kruskal(*groups)
        print(f"Kruskal-Wallis H-test for '{column}' by {group_col}: H-statistic = {h_statistic}, p-value = {p_kruskal}\n")

    def perform_repeated_measures_tests(df, column, subject_col="Participant ID", within_col="P"):
        """Run repeated-measures ANOVA, paired t-tests with Bonferroni, and Friedman test.

        df: DataFrame that contains rows for each subject-condition pair (subject_col, within_col).
        column: the dependent variable column name.
        """
        print(f"\n--- Repeated Measures Analysis for '{column}' ---")
        results = []
        # Prepare long-format data
        long = df[[subject_col, within_col, column]].dropna()
        if long.empty:
            print("No data available for repeated measures on column:", column)
            return results

        # Ensure each subject has measurements for all within levels by pivoting to wide and dropping missing
        try:
            wide = long.pivot(index=subject_col, columns=within_col, values=column)
        except Exception as e:
            print("Pivot failed:", e)
            return results

        # Drop subjects with missing values for any condition (complete-case analysis)
        wide = wide.dropna()
        if wide.shape[0] < 3 or wide.shape[1] < 2:
            print("Not enough complete cases for repeated-measures tests (need >=3 subjects and >=2 conditions).")
            return results

        # Convert wide back to long for pingouin (with complete cases only)
        long_complete = wide.reset_index().melt(id_vars=subject_col, var_name=within_col, value_name=column)

        # Repeated-measures ANOVA using pingouin
        try:
            aov = pg.rm_anova(data=long_complete, dv=column, within=within_col, subject=subject_col, detailed=True)
            print("Repeated-measures ANOVA (pingouin):")
            print(aov.to_string(index=False))
            try:
                row0 = aov.iloc[0]
                F = row0.get('F', np.nan)
                p_unc = row0.get('p-unc', row0.get('p', np.nan))
            except Exception:
                F = np.nan
                p_unc = np.nan
            results.append({
                'dependent': column,
                'test': 'rm_anova',
                'groupA': None,
                'groupB': None,
                'statistic': F,
                'p_uncorrected': p_unc,
                'p_corrected': None,
                'n_subjects': wide.shape[0],
                'n_conditions': wide.shape[1],
                'notes': ''
            })
        except Exception as e:
            print("rm_anova failed:", e)

        # Pairwise paired t-tests with Bonferroni correction (pingouin)
        try:
            pairwise = pg.pairwise_ttests(data=long_complete, dv=column, within=within_col, subject=subject_col, padjust='bonf')
            print("\nPaired t-tests with Bonferroni correction:")
            for _, prow in pairwise.iterrows():
                A = prow.get('A')
                B = prow.get('B')
                T = prow.get('T', prow.get('T-val', np.nan))
                p_unc = prow.get('p-unc', prow.get('p-val', prow.get('p', np.nan)))
                p_corr = prow.get('p-corr', prow.get('p-adjust', prow.get('p-bonf', np.nan)))
                results.append({
                    'dependent': column,
                    'test': 'paired_ttest',
                    'groupA': A,
                    'groupB': B,
                    'statistic': T,
                    'p_uncorrected': p_unc,
                    'p_corrected': p_corr,
                    'n_subjects': wide.shape[0],
                    'n_conditions': wide.shape[1],
                    'notes': ''
                })
            cols_to_show = [c for c in ['A', 'B', 'T', 'p-unc', 'p-corr'] if c in pairwise.columns]
            if cols_to_show:
                print(pairwise[cols_to_show].to_string(index=False))
        except Exception as e:
            print("pairwise_ttests failed:", e)

        # Friedman test (non-parametric repeated measures)
        try:
            # Friedman requires the arrays to be aligned across subjects; use wide columns
            groups = [wide[c].values for c in wide.columns]
            if len(groups) >= 2:
                stat, p = stats.friedmanchisquare(*groups)
                print(f"\nFriedman test: statistic = {stat:.4f}, p-value = {p:.4f}")
                # If significant, run pairwise Wilcoxon signed-rank tests with Bonferroni correction
                if p < 0.05 and len(groups) >= 2:
                    print("\nPost-hoc pairwise Wilcoxon tests (Bonferroni corrected):")
                    pairs = list(combinations(range(len(wide.columns)), 2))
                    pvals = []
                    names = []
                    for i, j in pairs:
                        a = groups[i]
                        b = groups[j]
                        try:
                            w_stat, p_w = stats.wilcoxon(a, b)
                        except Exception:
                            # If wilcoxon fails (e.g., zero differences), set p to 1
                            p_w = 1.0
                        pvals.append(p_w)
                        names.append((wide.columns[i], wide.columns[j]))
                    # Bonferroni correction
                    m = len(pvals)
                    for (c1, c2), pval in zip(names, pvals):
                        p_adj = min(pval * m, 1.0)
                        print(f"{c1} vs {c2}: raw p = {pval:.4f}, Bonferroni p = {p_adj:.4f}")
                        results.append({
                            'dependent': column,
                            'test': 'wilcoxon',
                            'groupA': c1,
                            'groupB': c2,
                            'statistic': np.nan,
                            'p_uncorrected': pval,
                            'p_corrected': p_adj,
                            'n_subjects': wide.shape[0],
                            'n_conditions': wide.shape[1],
                            'notes': ''
                        })
            else:
                print("Not enough groups for Friedman test")
        except Exception as e:
            print("Friedman test failed:", e)

        return results

    for threshold in [0.1, 0.3, 0.5, 0.7, 0.9]:
        for directory in ["mean"]:  # Add other directories as needed
            print(f"\n=== Processing threshold={threshold}, directory={directory} ===")
            subdirectory = f"./{threshold}/{directory}/"
            # Filter only valid users
            remaining_ids = pd.read_csv(subdirectory +"remaining_participant_ids.csv")
            remaining_ids_set = set(remaining_ids.iloc[:, 0])
            valid_users = [u for u in valid_users if u in remaining_ids_set]
            df_valid = data[data["Participant ID"].isin(valid_users)]
            cleaned_data = clean_data(df_valid)

            # collect results across thresholds/directories
            results_all = []
            print("Performing repeated-measures for 'mean_cultural_closeness':")
            results_all.extend(perform_repeated_measures_tests(cleaned_data, column="mean_cultural_closeness"))
            print("Performing repeated-measures for 'mean_competence':")
            results_all.extend(perform_repeated_measures_tests(cleaned_data, column="mean_competence"))
            for col in cols:
                print(f"Performing repeated-measures for '{col}':")
                results_all.extend(perform_repeated_measures_tests(cleaned_data, column=col))

            # After processing this threshold/directory, save results if any
            try:
                out_dir = os.path.dirname(__file__) if '__file__' in globals() else os.getcwd()
                out_path = os.path.join(out_dir, f'RMANOVA_{threshold}_{directory}.csv')
                if results_all:
                    df_results = pd.DataFrame(results_all)
                    df_results.to_csv(out_path, index=False)
                    print(f"Saved RMANOVA results to: {out_path}")
                else:
                    print("No results to save for this configuration.")
            except Exception as e:
                print("Failed to save RMANOVA.csv:", e)

    
