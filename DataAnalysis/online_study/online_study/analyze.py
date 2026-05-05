import re
import pandas as pd
import numpy as np
from scipy import stats
import matplotlib.pyplot as plt
import seaborn as sns
import json
import os

CSV_PATH = r"C:\Users\Utente\Desktop\CCN\DataAnalysis\online_study\online_study.csv"

# Results directories
BASE_DIR = os.path.dirname(CSV_PATH)
RESULTS_DIR = os.path.join(BASE_DIR, 'results')
CSV_DIR = os.path.join(RESULTS_DIR, 'csv')
JSON_DIR = os.path.join(RESULTS_DIR, 'json')
LATEX_DIR = os.path.join(RESULTS_DIR, 'latex')
PLOTS_DIR = os.path.join(RESULTS_DIR, 'plots')
os.makedirs(CSV_DIR, exist_ok=True)
os.makedirs(JSON_DIR, exist_ok=True)
os.makedirs(LATEX_DIR, exist_ok=True)
os.makedirs(PLOTS_DIR, exist_ok=True)


def save_csv_with_latex(df, path):
    """Save a CSV and also write a LaTeXified CSV variant to keep headers consistent.
    Overwrites the main CSV with the original df, and writes an additional
    LaTeX-friendly CSV alongside it with suffix '_latex.csv'."""
    try:
        df.to_csv(path, index=False)
    except Exception:
        # attempt to coerce object columns to strings then save
        try:
            df.astype(str).to_csv(path, index=False)
        except Exception:
            pass
    # write latexified variant
    try:
        df_l = latexify_df_for_export(df)
        base, ext = os.path.splitext(path)
        latex_path = f"{base}_latex.csv"
        df_l.to_csv(latex_path, index=False)
    except Exception:
        pass


def load_clean_dataframe(path):
    df = pd.read_csv(path, dtype=str)
    drop_patterns = [
        r"Informazioni cronologiche",
        r"Language/Lingua/Sprache",
        r"I, the participant",
        r"Io, il partecipante",
        r"Ich, teilnehmende Person",
        r"Voglio essere informato",
        r"Acconsento a partecipare",
        r"Ich m{0,1}\w* nicht.*Ergebnisse",
        r"bin mit der Teilnahme einverstanden",
    ]
    cols_to_drop = [c for c in df.columns if any(re.search(p, c, re.IGNORECASE) for p in drop_patterns)]
    df = df.drop(columns=cols_to_drop, errors="ignore")
    df = df.apply(
        lambda col: pd.to_numeric(col, errors="coerce")
        if pd.api.types.is_string_dtype(col) or col.dtype == object
        else col
    )
    return df


def get_language_rows(path):
    raw = pd.read_csv(path, nrows=0)
    all_columns = raw.columns.tolist()
    lang_col = next((c for c in all_columns if "Language" in c and "Lingua" in c and "Sprache" in c), None)
    if lang_col is None:
        raise ValueError("Language column not found")
    df = pd.read_csv(path, dtype=str)
    df = df.rename(columns={lang_col: "language"})
    return df


def get_language_block_columns(df):
    en = [
        c for c in df.columns
        if re.search(r"Which picture best describes|Rate how well the robot", c, flags=re.IGNORECASE)
    ]
    it = [
        c for c in df.columns
        if re.search(r"Quale immagine descrive|Valuta quanto bene il robot", c, flags=re.IGNORECASE)
    ]
    de = [
        c for c in df.columns
        if re.search(r"Welches Bild beschreibt|Bewerten Sie, wie gut der Roboter", c, flags=re.IGNORECASE)
    ]
    return en, it, de


def build_only_study_by_language(full_df, lang_df):
    en_cols, it_cols, de_cols = get_language_block_columns(full_df)
    rows = []

    for idx in full_df.index:
        lang_val = lang_df.loc[idx, "language"] if idx in lang_df.index else ""
        if isinstance(lang_val, str) and "English" in lang_val:
            cols = en_cols
        elif isinstance(lang_val, str) and "Italiano" in lang_val:
            cols = it_cols
        elif isinstance(lang_val, str) and "Deutsch" in lang_val:
            cols = de_cols
        else:
            cols = en_cols

        selected = full_df.loc[idx, cols]
        if selected.isna().all():
            continue

        # Normalize question names and disambiguate duplicates (..1, ..2)
        cleaned = []
        dup_counts = {}
        for c in cols:
            base = re.sub(r"\s*\.\.\d+$", "", c).strip()
            dup_counts[base] = dup_counts.get(base, 0) + 1
            if dup_counts[base] > 1:
                cleaned_name = f"{base}__{dup_counts[base]}"
            else:
                cleaned_name = base
            cleaned.append(cleaned_name)

        selected.index = cleaned
        selected.name = idx
        rows.append(selected)

    if len(rows) == 0:
        return pd.DataFrame()
    result = pd.DataFrame(rows)
    result.index = [r.name for r in rows]
    return result


def summarize_numeric(numeric_df):
    if numeric_df.empty:
        return pd.DataFrame()
    return numeric_df.describe().T[["count", "mean", "std", "min", "25%", "50%", "75%", "max"]]


def cohen_d(a, b):
    a = np.asarray(a)
    b = np.asarray(b)
    na, nb = len(a), len(b)
    if na < 2 or nb < 2:
        return np.nan
    ma, mb = np.nanmean(a), np.nanmean(b)
    sa, sb = np.nanstd(a, ddof=1), np.nanstd(b, ddof=1)
    # pooled sd
    sd_pooled = np.sqrt(((na - 1) * sa ** 2 + (nb - 1) * sb ** 2) / (na + nb - 2))
    return (ma - mb) / sd_pooled if sd_pooled > 0 else np.nan


def bh_adjust(pvals):
    """Benjamini-Hochberg FDR adjustment for an array-like of p-values."""
    p = np.asarray(pvals, dtype=float)
    n = len(p)
    # preserve nan positions
    nan_mask = np.isnan(p)
    idx = np.arange(n)
    sorted_idx = np.argsort(p, kind='mergesort')
    sorted_p = p[sorted_idx]
    adjusted = np.empty(n, dtype=float)
    adjusted.fill(np.nan)
    if n - nan_mask.sum() == 0:
        return adjusted
    m = n - nan_mask.sum()
    # compute BH adjusted
    ranks = np.arange(1, m + 1)
    valid_idx = sorted_idx[~nan_mask[sorted_idx]]
    valid_p = p[valid_idx]
    order = np.argsort(valid_p)
    sorted_valid_p = valid_p[order]
    sorted_idx_valid = valid_idx[order]
    bh = (sorted_valid_p * m) / (np.arange(1, len(sorted_valid_p) + 1))
    # ensure monotonicity
    bh_monotonic = np.minimum.accumulate(bh[::-1])[::-1]
    adjusted[sorted_idx_valid] = np.minimum(bh_monotonic, 1.0)
    return adjusted


def ttest_between_languages(dataset, lang_df, question_cols):
    # detect languages from the language column (any value, not only fixed three)
    available = lang_df["language"].dropna().unique().tolist()
    groups = {}
    for lang in available:
        idx = lang_df[lang_df["language"].str.contains(re.escape(lang), na=False, case=False)].index
        groups[lang] = dataset.loc[idx, question_cols].stack().dropna().values

    results = {}
    for i in range(len(available)):
        for j in range(i + 1, len(available)):
            a = available[i]
            b = available[j]
            ga = groups.get(a, [])
            gb = groups.get(b, [])
            if len(ga) >= 2 and len(gb) >= 2:
                # run diagnostics and compute both tests for robustness
                diag = run_diagnostic_tests(ga, gb)
                equal_var = diag.get("levene_p", 0) > 0.05
                try:
                    t_stat, t_p = stats.ttest_ind(ga, gb, nan_policy="omit", equal_var=equal_var)
                except Exception:
                    t_stat, t_p = np.nan, np.nan
                try:
                    u_stat, u_p = stats.mannwhitneyu(ga, gb, alternative="two-sided")
                except Exception:
                    u_stat, u_p = np.nan, np.nan
                d = cohen_d(ga, gb)
                # Always prefer the less-assumptions test (Mann-Whitney) as primary
                primary = "mannwhitney"
                results[f"{a} vs {b}"] = {
                    "primary_test": primary,
                    "t_stat": float(t_stat) if not np.isnan(t_stat) else np.nan,
                    "t_p": float(t_p) if not np.isnan(t_p) else np.nan,
                    "mw_u": float(u_stat) if not np.isnan(u_stat) else np.nan,
                    "mw_p": float(u_p) if not np.isnan(u_p) else np.nan,
                    "cohen_d": float(d) if not np.isnan(d) else np.nan,
                    "n_a": len(ga),
                    "n_b": len(gb),
                    "diag": diag,
                }
    return results


def run_diagnostic_tests(vals_a, vals_b):
    """
    Run Shapiro-Wilk (normality), Levene (variance equality), and Mann-Whitney U tests.
    Returns dict with all test results.
    """
    results = {
        "shapiro_a_p": np.nan,
        "shapiro_b_p": np.nan,
        "levene_p": np.nan,
        "mannwhitney_u": np.nan,
        "mannwhitney_p": np.nan,
        "assumptions_ok": None,
    }
    
    if len(vals_a) < 3 or len(vals_b) < 3:
        return results
    
    # Shapiro-Wilk test for normality (H0: data is normal)
    try:
        _, p_a = stats.shapiro(vals_a)
        results["shapiro_a_p"] = float(p_a)
    except:
        pass
    
    try:
        _, p_b = stats.shapiro(vals_b)
        results["shapiro_b_p"] = float(p_b)
    except:
        pass
    
    # Levene's test for equal variances (H0: variances are equal)
    try:
        _, p_levene = stats.levene(vals_a, vals_b)
        results["levene_p"] = float(p_levene)
    except:
        pass
    
    # Mann-Whitney U test (non-parametric alternative to t-test)
    try:
        u_stat, p_mw = stats.mannwhitneyu(vals_a, vals_b, alternative="two-sided")
        results["mannwhitney_u"] = float(u_stat)
        results["mannwhitney_p"] = float(p_mw)
    except:
        pass
    
    # Simple recommendation: both groups normal AND equal variance → t-test safe
    norm_a = results["shapiro_a_p"] > 0.05 if not np.isnan(results["shapiro_a_p"]) else False
    norm_b = results["shapiro_b_p"] > 0.05 if not np.isnan(results["shapiro_b_p"]) else False
    eq_var = results["levene_p"] > 0.05 if not np.isnan(results["levene_p"]) else False
    results["assumptions_ok"] = norm_a and norm_b and eq_var
    
    return results


def compare_corresponding_questions(df, lang_df, lang_a, lang_b):
    # choose columns by block per language
    all_en, all_it, all_de = get_language_block_columns(df)
    block_map = {
        "English": all_en,
        "Italiano": all_it,
        "Deutsch": all_de,
    }
    if lang_a not in block_map or lang_b not in block_map:
        print(f"Language block not found for {lang_a} or {lang_b}")
        return pd.DataFrame()

    cols_a = block_map[lang_a]
    cols_b = block_map[lang_b]
    n_items = min(len(cols_a), len(cols_b))

    idx_a = lang_df[lang_df["language"].str.contains(re.escape(lang_a), na=False, case=False)].index
    idx_b = lang_df[lang_df["language"].str.contains(re.escape(lang_b), na=False, case=False)].index

    rows = []
    for i in range(n_items):
        col_a = cols_a[i]
        col_b = cols_b[i]
        vals_a = pd.to_numeric(df.loc[idx_a, col_a], errors="coerce").dropna()
        vals_b = pd.to_numeric(df.loc[idx_b, col_b], errors="coerce").dropna()

        # initialize raw test variables to nan so they are always available
        t_raw_stat = np.nan
        t_raw_p = np.nan
        mw_raw_u = np.nan
        mw_raw_p = np.nan
        d_val = np.nan

        if len(vals_a) < 2 or len(vals_b) < 2:
            t, p = np.nan, np.nan
            diag = run_diagnostic_tests(vals_a.values, vals_b.values)
            test_used = None
            chosen_p = np.nan
            chosen_stat = np.nan
        else:
            # Run diagnostic tests and compute both tests for consistency
            diag = run_diagnostic_tests(vals_a.values, vals_b.values)
            equal_var = diag.get("levene_p", 0) > 0.05
            try:
                t_stat, p_val = stats.ttest_ind(vals_a, vals_b, nan_policy="omit", equal_var=equal_var)
            except Exception:
                t_stat, p_val = np.nan, np.nan
            try:
                u_stat, p_mw = stats.mannwhitneyu(vals_a, vals_b, alternative="two-sided")
            except Exception:
                u_stat, p_mw = np.nan, np.nan
            d_val = cohen_d(vals_a.values, vals_b.values)
            # keep both raw values first
            t_raw_p = float(p_val) if not np.isnan(p_val) else np.nan
            t_raw_stat = float(t_stat) if not np.isnan(t_stat) else np.nan
            mw_raw_u = float(u_stat) if not np.isnan(u_stat) else np.nan
            mw_raw_p = float(p_mw) if not np.isnan(p_mw) else np.nan
            # Use the less-assumptions test (Mann-Whitney) as primary and set chosen stats
            test_used = "mannwhitney"
            chosen_stat = mw_raw_u
            chosen_p = mw_raw_p

        # Append row with diagnostics and both-test outcomes
        rows.append({
            "question_index": i + 1,
            f"{lang_a}_col": col_a,
            f"{lang_b}_col": col_b,
            f"{lang_a}_mean": float(vals_a.mean()) if len(vals_a) else np.nan,
            f"{lang_b}_mean": float(vals_b.mean()) if len(vals_b) else np.nan,
            f"{lang_a}_std": float(vals_a.std(ddof=1)) if len(vals_a) else np.nan,
            f"{lang_b}_std": float(vals_b.std(ddof=1)) if len(vals_b) else np.nan,
            f"{lang_a}_n": len(vals_a),
            f"{lang_b}_n": len(vals_b),
            "t_stat": t_raw_stat if 't_raw_stat' in locals() else np.nan,
            "t_test_p": t_raw_p if 't_raw_p' in locals() else np.nan,
            "mean_diff": float(vals_a.mean() - vals_b.mean()) if len(vals_a) and len(vals_b) else np.nan,
            "shapiro_a_p": diag["shapiro_a_p"],
            "shapiro_b_p": diag["shapiro_b_p"],
            "levene_p": diag["levene_p"],
            "mannwhitney_u": mw_raw_u if 'mw_raw_u' in locals() else diag["mannwhitney_u"],
            "mannwhitney_p": mw_raw_p if 'mw_raw_p' in locals() else diag["mannwhitney_p"],
            "assumptions_met": diag["assumptions_ok"],
            "chosen_test": test_used,
            "chosen_stat": chosen_stat,
            "chosen_p": chosen_p,
            "cohen_d": float(d_val) if 'd_val' in locals() and not np.isnan(d_val) else np.nan,
            "t_test_p_adj": np.nan,
            "mannwhitney_p_adj": np.nan,
        })

    out = pd.DataFrame(rows)
    # Apply BH adjustment to both sets of p-values across questions
    try:
        out['t_test_p_adj'] = bh_adjust(out['t_test_p'].values)
    except Exception:
        out['t_test_p_adj'] = np.nan
    try:
        out['mannwhitney_p_adj'] = bh_adjust(out['mannwhitney_p'].values)
    except Exception:
        out['mannwhitney_p_adj'] = np.nan
    out_path = os.path.join(CSV_DIR, 'corresponding_question_comparison.csv')
    save_csv_with_latex(out, out_path)
    return out


def plot_corresponding_pairs(df, lang_df, lang_a, lang_b):
    # per-position matching: 1st item italian vs 1st german, 2nd vs 2nd ...
    _, cols_it, cols_de = get_language_block_columns(df)
    n_items = min(len(cols_it), len(cols_de))

    idx_it = lang_df[lang_df["language"].str.contains(re.escape(lang_a), na=False, case=False)].index
    idx_de = lang_df[lang_df["language"].str.contains(re.escape(lang_b), na=False, case=False)].index

    rows = []
    for i in range(n_items):
        col_it = cols_it[i]
        col_de = cols_de[i]

        vals_it = pd.to_numeric(df.loc[idx_it, col_it], errors="coerce").dropna()
        vals_de = pd.to_numeric(df.loc[idx_de, col_de], errors="coerce").dropna()

        rows.append(pd.DataFrame({
            "question_index": i + 1,
            "language": lang_a,
            "value": vals_it,
        }))
        rows.append(pd.DataFrame({
            "question_index": i + 1,
            "language": lang_b,
            "value": vals_de,
        }))

    if not rows:
        print("No data to plot for corresponding pairs")
        return

    concat = pd.concat(rows, ignore_index=True)
    plt.figure(figsize=(10, 6))
    sns.violinplot(
        x="question_index",
        y="value",
        hue="language",
        data=concat,
        split=False,
        inner="quartile",
        scale="width",
        palette="Set2",
        dodge=True,
        cut=0,
        bw=0.2,
    )
    plt.xlabel("Question index (same position in each language block)")
    plt.ylabel("Rating")
    plt.title(f"Corresponding question pair distributions: {lang_a} vs {lang_b}")
    plt.ylim(1, 7)
    plt.tight_layout()

    out_png = r"C:\Users\Utente\Desktop\CCN\DataAnalysis\online_study\corresponding_pairs_distribution.png"
    plt.savefig(out_png, dpi=150)
    plt.close()
    print(f"Saved paired distribution plot: {out_png}")

    # Also save each question index as a separate figure for fixed pair comparison
    question_ids = concat['question_index'].unique()
    for qid in question_ids:
        subset = concat[concat['question_index'] == qid]
        plt.figure(figsize=(5, 4))
        sns.violinplot(
            x='language',
            y='value',
            data=subset,
            split=False,
            inner='quartile',
            scale='width',
            palette='Set2',
            dodge=True,
        )
        plt.ylim(1, 7)
        plt.xlabel(f'Question index {qid}')
        plt.title(f'Position {qid}: {lang_a} vs {lang_b}')
        plt.tight_layout()
        per_png = fr"C:\Users\Utente\Desktop\CCN\DataAnalysis\online_study\corresponding_pair_q{qid}.png"
        plt.savefig(per_png, dpi=150)
        plt.close()
        print(f"Saved per-question pair plot: {per_png}")


def plot_distribution_per_lang(df, lang_df, question_cols):
    # dynamic languages for improved generalization
    langs = lang_df["language"].dropna().unique().tolist()
    plot_data = []
    for lang in langs:
        idx = lang_df[lang_df["language"].str.contains(re.escape(lang), na=False, case=False)].index
        subset = df.loc[idx, question_cols]
        long = subset.melt(var_name="question", value_name="value")
        long["language"] = lang
        plot_data.append(long)
    all_long = pd.concat(plot_data, ignore_index=True)
    all_long = all_long.dropna(subset=["value"])
    if all_long.empty:
        print("No numeric data to plot.")
        return

    # For fair comparison, use violinplot + boxplot overlay with dodge for language buckets
    plt.figure(figsize=(16, 8))
    sns.violinplot(
        x="question",
        y="value",
        hue="language",
        data=all_long,
        split=False,
        inner=None,
        cut=0,
        scale="width",
        palette="Set2",
        dodge=True,
    )

    sns.boxplot(
        x="question",
        y="value",
        hue="language",
        data=all_long,
        showcaps=True,
        boxprops={"facecolor": "none"},
        showfliers=False,
        whiskerprops={"linewidth": 1},
        dodge=True,
        palette="dark",
    )

    plt.xticks(rotation=45, ha="right")
    plt.ylabel("Rating")
    plt.ylim(0.5, 7.5)
    plt.title("Fair per-question distribution by language (only_study)")
    plt.tight_layout()

    # remove duplicate legend entries from overlaying plots
    handles, labels = plt.gca().get_legend_handles_labels()
    by_label = dict(zip(labels, handles))
    plt.legend(by_label.values(), by_label.keys(), title="language", bbox_to_anchor=(1.02, 1), loc="upper left")

    out_png = r"C:\Users\Utente\Desktop\CCN\DataAnalysis\online_study\only_study_distribution.png"
    plt.savefig(out_png, dpi=150)
    plt.close()
    print(f"Saved plot: {out_png}")


def main():
    full_df = load_clean_dataframe(CSV_PATH)
    lang_df = get_language_rows(CSV_PATH)

    # Exclude English respondents if not interested
    english_mask = lang_df["language"].str.contains("English", na=False, case=False)
    # preserve the original indices for accurate subsetting of full_df
    lang_df_filtered = lang_df[~english_mask].copy()
    full_df = full_df.loc[lang_df_filtered.index].reset_index(drop=True)
    # now reset index of lang_df to align with full_df rows
    lang_df = lang_df_filtered.reset_index(drop=True)

    only_study_df = build_only_study_by_language(full_df, lang_df)
    lang_df = lang_df.loc[only_study_df.index]

    print("\n===Column selection for only_study===")
    print(only_study_df.columns.tolist())

    langs = lang_df["language"].dropna().unique().tolist()

    stats_by_lang = {}
    for lang in langs:
        idx = lang_df[lang_df["language"].str.contains(re.escape(lang), na=False, case=False)].index
        subset = only_study_df.loc[idx].select_dtypes(include=[np.number])
        stats_by_lang[lang] = summarize_numeric(subset)

    print("\n=== per-language numeric summary ===")
    for lang, stat in stats_by_lang.items():
        n = len(lang_df[lang_df["language"].str.contains(lang, na=False)])
        print(f"\n--- {lang} ({n} rows) ---")
        if stat.empty:
            print("No numeric columns found for this language")
        else:
            print(stat)

    question_cols = only_study_df.select_dtypes(include=[np.number]).columns.tolist()
    ttest_results = ttest_between_languages(only_study_df, lang_df, question_cols)
    # Convert per-language comparison results to DataFrame and adjust p-values
    try:
        pl_rows = []
        for k, v in ttest_results.items():
            pl_rows.append({
                'comparison': k,
                'primary_test': v.get('primary_test'),
                't_stat': v.get('t_stat'),
                't_p': v.get('t_p'),
                'mw_u': v.get('mw_u'),
                'mw_p': v.get('mw_p'),
                'cohen_d': v.get('cohen_d'),
                'n_a': v.get('n_a'),
                'n_b': v.get('n_b'),
            })
        per_lang_df = pd.DataFrame(pl_rows) if pl_rows else None
        if per_lang_df is not None and not per_lang_df.empty:
            per_lang_df['t_p_adj'] = bh_adjust(per_lang_df['t_p'].values)
            per_lang_df['mw_p_adj'] = bh_adjust(per_lang_df['mw_p'].values)
            per_lang_csv = os.path.join(CSV_DIR, 'per_language_comparisons_flat.csv')
            save_csv_with_latex(per_lang_df, per_lang_csv)
    except Exception:
        per_lang_df = None
    print("\n=== per-language comparisons (tests chosen by diagnostics) ===")
    if not ttest_results:
        print("No comparisons possible, insufficient numeric values.")
    for k, v in ttest_results.items():
        test_used = v.get("test_used", "unknown")
        if test_used == "t-test":
            t = v.get("t", np.nan)
            p = v.get("p", np.nan)
            print(f"{k}: test=t-test, t={t:.3f}, p={p:.3f}, n1={v.get('n_a')} n2={v.get('n_b')}")
        elif test_used == "mannwhitney":
            u = v.get("u", np.nan)
            p = v.get("p", np.nan)
            print(f"{k}: test=mannwhitney, U={u:.3f}, p={p:.3f}, n1={v.get('n_a')} n2={v.get('n_b')}")
        else:
            p = v.get("p", np.nan)
            try:
                print(f"{k}: test={test_used}, p={p:.3f}")
            except Exception:
                print(f"{k}: test={test_used}, p={p}")

    print("\n=== compare corresponding questions by position English filtered-out block ===")
    print("(Includes normality, variance equality tests + Mann-Whitney U as robust alternative)")
    comp_df = compare_corresponding_questions(only_study_df, lang_df, "Italiano", "Deutsch")
    # Create a reusable question index (q1, q2, ...) mapping to long question text
    if not comp_df.empty:
        try:
            questions_index_df = comp_df[["question_index", "Italiano_col", "Deutsch_col"]].drop_duplicates().sort_values("question_index")
            questions_index_df = questions_index_df.reset_index(drop=True)
            questions_index_df["q_label"] = questions_index_df["question_index"].apply(lambda x: f"q{int(x)}")
            q_index_csv = os.path.join(CSV_DIR, 'questions_index.csv')
            save_csv_with_latex(questions_index_df, q_index_csv)
            # Defer LaTeX export and summary bookkeeping until later (summary created further down)
            # add q_label to comp_df for downstream tables
            comp_df = comp_df.merge(questions_index_df[['question_index', 'q_label']], on='question_index', how='left')
        except Exception as e:
            print(f"Failed to create questions index: {e}")
    
    # Print diagnostic summary
    print("\n=== ASSUMPTION DIAGNOSTIC SUMMARY ===")
    n_assumptions_met = comp_df["assumptions_met"].sum()
    n_total = len(comp_df)
    print(f"Assumptions met (normality + equal variance): {n_assumptions_met}/{n_total}")
    print(f"\nPositions where assumptions VIOLATED (use Mann-Whitney U):")
    violated = comp_df[~comp_df["assumptions_met"]]
    if not violated.empty:
        print(violated[["question_index", "shapiro_a_p", "shapiro_b_p", "levene_p", "mannwhitney_p"]].to_string(index=False))
    else:
        print("  None - all positions meet assumptions")
    
    print("\nFirst 12 rows of full diagnostic results:")
    print(comp_df[["question_index", "t_test_p", "mannwhitney_p", "shapiro_a_p", "shapiro_b_p", "levene_p", "assumptions_met"]].head(12).to_string(index=False))

    # Add Cohen's d and significant pairs summary
    def cohen_d(a, b):
        na, nb = len(a), len(b)
        if na < 2 or nb < 2:
            return np.nan
        mean_diff = a.mean() - b.mean()
        sd_pooled = np.sqrt(((na - 1) * a.std(ddof=1) ** 2 + (nb - 1) * b.std(ddof=1) ** 2) / (na + nb - 2))
        return mean_diff / sd_pooled if sd_pooled > 0 else np.nan

    print("\n=== SIGNIFICANT DIFFERENCES ===")
    print("\nUsing ROBUST test (Mann-Whitney U, p<0.05):")
    sign_rows_robust = []
    for _, row in comp_df.iterrows():
        if not np.isnan(row['mannwhitney_p']) and row['mannwhitney_p'] < 0.05:
            v_a = pd.to_numeric(only_study_df.loc[lang_df[lang_df['language'].str.contains('Italiano', na=False)].index, row['Italiano_col']], errors='coerce').dropna()
            v_b = pd.to_numeric(only_study_df.loc[lang_df[lang_df['language'].str.contains('Deutsch', na=False)].index, row['Deutsch_col']], errors='coerce').dropna()
            d = cohen_d(v_a, v_b)
            sign_rows_robust.append({
                'question_index': row['question_index'],
                'italiano_col': row['Italiano_col'],
                'deutsch_col': row['Deutsch_col'],
                'mean_diff': row['mean_diff'],
                'mannwhitney_p': row['mannwhitney_p'],
                'cohen_d': d,
                'assumptions_met': row['assumptions_met'],
            })

    if sign_rows_robust:
        significant_robust_df = pd.DataFrame(sign_rows_robust)
        significant_csv = r"C:\Users\Utente\Desktop\CCN\DataAnalysis\online_study\significant_corresponding_comparisons_robust.csv"
        save_csv_with_latex(significant_robust_df, significant_csv)
        print(f"Found {len(significant_robust_df)} significant differences")
        print(significant_robust_df.to_string(index=False))
        print(f"\nSaved to: {significant_csv}")
    else:
        print("No significant positional comparisons (Mann-Whitney U, p<0.05) found.")
    
    print("\nUsing PARAMETRIC test (Welch's t-test, p<0.05, for reference):")
    sign_rows_ttest = []
    for _, row in comp_df.iterrows():
        if not np.isnan(row['t_test_p']) and row['t_test_p'] < 0.05:
            v_a = pd.to_numeric(only_study_df.loc[lang_df[lang_df['language'].str.contains('Italiano', na=False)].index, row['Italiano_col']], errors='coerce').dropna()
            v_b = pd.to_numeric(only_study_df.loc[lang_df[lang_df['language'].str.contains('Deutsch', na=False)].index, row['Deutsch_col']], errors='coerce').dropna()
            d = cohen_d(v_a, v_b)
            sign_rows_ttest.append({
                'question_index': row['question_index'],
                'italiano_col': row['Italiano_col'],
                'deutsch_col': row['Deutsch_col'],
                'mean_diff': row['mean_diff'],
                't_test_p': row['t_test_p'],
                'cohen_d': d,
                'assumptions_met': row['assumptions_met'],
            })
    
    if sign_rows_ttest:
        print(f"Found {len(sign_rows_ttest)} significant differences (⚠️ use with caution if assumptions violated)")
        print(pd.DataFrame(sign_rows_ttest).to_string(index=False))
    else:
        print("No significant positional comparisons (Welch's t-test, p<0.05).")

    #plot_corresponding_pairs(only_study_df, lang_df, "Italiano", "Deutsch")
    #plot_distribution_per_lang(only_study_df, lang_df, question_cols)

    out_path = os.path.join(CSV_DIR, 'only_study_cleaned.csv')
    save_csv_with_latex(only_study_df, out_path)
    print(f"\nSaved cleaned only_study CSV to: {out_path}")
    print(f"Saved comparing related questions to {os.path.join(CSV_DIR, 'corresponding_question_comparison.csv')}")

    # Save a comprehensive JSON summary of the analysis
    summary = {}
    # stats_by_lang: convert DataFrame to dicts
    summary['stats_by_lang'] = {}
    for lang, dfstat in stats_by_lang.items():
        try:
            summary['stats_by_lang'][lang] = dfstat.fillna('').to_dict(orient='index') if isinstance(dfstat, pd.DataFrame) else {}
        except Exception:
            summary['stats_by_lang'][lang] = {}

    # per-language comparison results
    summary['per_language_comparisons'] = ttest_results

    # corresponding question diagnostics (comp_df)
    try:
        summary['corresponding_questions'] = comp_df.fillna('').to_dict(orient='records')
    except Exception:
        summary['corresponding_questions'] = []

    # significant lists
    try:
        summary['significant_robust'] = significant_robust_df.fillna('').to_dict(orient='records') if 'significant_robust_df' in locals() else []
    except Exception:
        summary['significant_robust'] = []

    try:
        summary['significant_parametric'] = sign_rows_ttest if 'sign_rows_ttest' in locals() else []
    except Exception:
        summary['significant_parametric'] = []

    summary['saved_files'] = {
        'only_study_cleaned': out_path,
        'corresponding_question_comparison': os.path.join(CSV_DIR, 'corresponding_question_comparison.csv'),
    }

    json_path = os.path.join(JSON_DIR, 'analysis_summary.json')
    try:
        with open(json_path, 'w', encoding='utf-8') as jf:
            json.dump(summary, jf, ensure_ascii=False, indent=2)
        print(f"Saved analysis summary JSON to: {json_path}")
    except Exception as e:
        print(f"Failed to save analysis summary JSON: {e}")

    # Also write a flattened CSV of per-language comparisons for quick inspection
    try:
        rows = []
        for k, v in ttest_results.items():
            r = {'comparison': k, 'test_used': v.get('test_used')}
            r.update({kk: vv for kk, vv in v.items() if kk not in ['diag', 'test_used']})
            rows.append(r)
        perlang_tmp = pd.DataFrame(rows)
        save_csv_with_latex(perlang_tmp, os.path.join(CSV_DIR, 'per_language_comparisons_flat.csv'))
        summary['saved_files']['per_language_comparisons_flat'] = os.path.join(CSV_DIR, 'per_language_comparisons_flat.csv')
        print("Saved flattened per-language comparisons CSV.")
    except Exception as e:
        print(f"Failed to save flattened comparisons CSV: {e}")

    # --- Grouped-construct aggregation and comparisons ---
    cols_all = only_study_df.columns.tolist()

    # Exclusive grouping requested by user: culture (q1,q2,q3), videos, images (photos)
    # Build culture cols using the question mapping from comp_df (question_index 1-3)
    culture_cols_it = []
    culture_cols_de = []
    if not comp_df.empty:
        for _, r in comp_df.iterrows():
            qi = int(r['question_index'])
            if qi in (1, 2, 3):
                if r.get('Italiano_col'):
                    culture_cols_it.append(r.get('Italiano_col'))
                if r.get('Deutsch_col'):
                    culture_cols_de.append(r.get('Deutsch_col'))

    # Build exclusive groups based on question indices provided by comp_df
    # User mapping: culture = q1..q3, videos = q4..q9, images = q10..q15
    video_idx = set(range(4, 10))
    image_idx = set(range(10, 16))

    video_cols_it = []
    video_cols_de = []
    image_cols_it = []
    image_cols_de = []

    if not comp_df.empty:
        for _, r in comp_df.iterrows():
            try:
                qi = int(r['question_index'])
            except Exception:
                continue
            if qi in video_idx:
                if r.get('Italiano_col'):
                    video_cols_it.append(r.get('Italiano_col'))
                if r.get('Deutsch_col'):
                    video_cols_de.append(r.get('Deutsch_col'))
            if qi in image_idx:
                if r.get('Italiano_col'):
                    image_cols_it.append(r.get('Italiano_col'))
                if r.get('Deutsch_col'):
                    image_cols_de.append(r.get('Deutsch_col'))

    # Ensure uniqueness and that columns exist in only_study_df
    video_cols_it = [c for c in dict.fromkeys(video_cols_it) if c in only_study_df.columns]
    video_cols_de = [c for c in dict.fromkeys(video_cols_de) if c in only_study_df.columns]
    image_cols_it = [c for c in dict.fromkeys(image_cols_it) if c in only_study_df.columns]
    image_cols_de = [c for c in dict.fromkeys(image_cols_de) if c in only_study_df.columns]

    # Build group rows for exactly the three groups
    group_rows = []
    groups_spec = [
        ('culture', culture_cols_it, culture_cols_de),
        ('videos', video_cols_it, video_cols_de),
        ('images', image_cols_it, image_cols_de),
    ]

    idx_it = lang_df[lang_df['language'].str.contains('Italiano', na=False)].index
    idx_de = lang_df[lang_df['language'].str.contains('Deutsch', na=False)].index

    for gname, cols_it, cols_de in groups_spec:
        it_scores = None
        de_scores = None
        if cols_it:
            it_df = only_study_df.loc[idx_it, [c for c in cols_it if c in only_study_df.columns]].apply(pd.to_numeric, errors='coerce')
            if not it_df.empty:
                it_scores = it_df.mean(axis=1, skipna=True).dropna()
        if cols_de:
            de_df = only_study_df.loc[idx_de, [c for c in cols_de if c in only_study_df.columns]].apply(pd.to_numeric, errors='coerce')
            if not de_df.empty:
                de_scores = de_df.mean(axis=1, skipna=True).dropna()

        diag = None
        chosen = {'test': None, 'stat': np.nan, 'p': np.nan, 'n_it': 0, 'n_de': 0}
        if it_scores is not None and de_scores is not None and len(it_scores) >= 2 and len(de_scores) >= 2:
            diag = run_diagnostic_tests(it_scores.values, de_scores.values)
            if diag.get('assumptions_ok'):
                equal_var = diag.get('levene_p', 0) > 0.05
                t_stat, p_val = stats.ttest_ind(it_scores, de_scores, nan_policy='omit', equal_var=equal_var)
                chosen.update({'test': 't-test', 'stat': float(t_stat), 'p': float(p_val), 'n_it': len(it_scores), 'n_de': len(de_scores)})
            else:
                try:
                    u_stat, p_mw = stats.mannwhitneyu(it_scores.values, de_scores.values, alternative='two-sided')
                except Exception:
                    u_stat, p_mw = np.nan, np.nan
                chosen.update({'test': 'mannwhitney', 'stat': float(u_stat) if not np.isnan(u_stat) else np.nan, 'p': float(p_mw) if not np.isnan(p_mw) else np.nan, 'n_it': len(it_scores), 'n_de': len(de_scores)})

        try:
            d = cohen_d(it_scores, de_scores) if it_scores is not None and de_scores is not None else np.nan
        except Exception:
            d = np.nan

        group_rows.append({
            'group': gname,
            'italiano_cols': cols_it,
            'deutsch_cols': cols_de,
            'n_italiano': chosen.get('n_it', 0),
            'n_deutsch': chosen.get('n_de', 0),
            'test': chosen.get('test'),
            'stat': chosen.get('stat'),
            'p': chosen.get('p'),
            'cohen_d': d,
            'diagnostics': diag,
        })

    group_df = pd.DataFrame(group_rows)
    group_csv = os.path.join(CSV_DIR, 'group_comparisons_auto.csv')
    group_json = os.path.join(JSON_DIR, 'group_comparisons_auto.json')
    try:
        save_csv_with_latex(group_df, group_csv)
        with open(group_json, 'w', encoding='utf-8') as gf:
            json.dump(group_rows, gf, ensure_ascii=False, indent=2)
        print(f"Saved automatic group comparison CSV: {group_csv}")
        print(f"Saved automatic group comparison JSON: {group_json}")
        # update summary later (summary exists further down)
    except Exception as e:
        print(f"Failed to save automatic group comparisons: {e}")

    # --- Culture aggregation (q1,q2,q3) ---
    # Map question indices to columns from comp_df
    culture_cols_it = []
    culture_cols_de = []
    if not comp_df.empty:
        for _, r in comp_df.iterrows():
            qi = int(r['question_index'])
            if qi in (1, 2, 3):
                if r.get('Italiano_col'):
                    culture_cols_it.append(r.get('Italiano_col'))
                if r.get('Deutsch_col'):
                    culture_cols_de.append(r.get('Deutsch_col'))

    # compute per-respondent culture score (mean of q1..q3 in their language block)
    idx_it = lang_df[lang_df['language'].str.contains('Italiano', na=False)].index
    idx_de = lang_df[lang_df['language'].str.contains('Deutsch', na=False)].index

    culture_it = None
    culture_de = None
    if culture_cols_it:
        it_df = only_study_df.loc[idx_it, culture_cols_it].apply(pd.to_numeric, errors='coerce')
        culture_it = it_df.mean(axis=1, skipna=True).dropna()
    if culture_cols_de:
        de_df = only_study_df.loc[idx_de, culture_cols_de].apply(pd.to_numeric, errors='coerce')
        culture_de = de_df.mean(axis=1, skipna=True).dropna()

    culture_summary = {}
    try:
        culture_summary['italiano'] = {
            'n': int(len(culture_it)) if culture_it is not None else 0,
            'mean': float(culture_it.mean()) if culture_it is not None and len(culture_it) else np.nan,
            'std': float(culture_it.std(ddof=1)) if culture_it is not None and len(culture_it) > 1 else np.nan,
            'median': float(culture_it.median()) if culture_it is not None and len(culture_it) else np.nan,
        }
    except Exception:
        culture_summary['italiano'] = {}
    try:
        culture_summary['deutsch'] = {
            'n': int(len(culture_de)) if culture_de is not None else 0,
            'mean': float(culture_de.mean()) if culture_de is not None and len(culture_de) else np.nan,
            'std': float(culture_de.std(ddof=1)) if culture_de is not None and len(culture_de) > 1 else np.nan,
            'median': float(culture_de.median()) if culture_de is not None and len(culture_de) else np.nan,
        }
    except Exception:
        culture_summary['deutsch'] = {}

    # test for culture group
    culture_test = {'test': None, 'stat': np.nan, 'p': np.nan}
    if culture_it is not None and culture_de is not None and len(culture_it) >= 2 and len(culture_de) >= 2:
        diag = run_diagnostic_tests(culture_it.values, culture_de.values)
        if diag.get('assumptions_ok'):
            t_stat, p_val = stats.ttest_ind(culture_it, culture_de, nan_policy='omit', equal_var=(diag.get('levene_p',0)>0.05))
            culture_test.update({'test': 't-test', 'stat': float(t_stat), 'p': float(p_val)})
        else:
            try:
                u_stat, p_mw = stats.mannwhitneyu(culture_it.values, culture_de.values, alternative='two-sided')
            except Exception:
                u_stat, p_mw = np.nan, np.nan
            culture_test.update({'test': 'mannwhitney', 'stat': float(u_stat) if not np.isnan(u_stat) else np.nan, 'p': float(p_mw) if not np.isnan(p_mw) else np.nan})

    # save culture summary
    culture_out = {
        'cols_italiano': culture_cols_it,
        'cols_deutsch': culture_cols_de,
        'summary': culture_summary,
        'test': culture_test,
    }
    culture_json = os.path.join(JSON_DIR, 'culture_summary.json')
    culture_csv = os.path.join(CSV_DIR, 'culture_summary.csv')
    try:
        with open(culture_json, 'w', encoding='utf-8') as cf:
            json.dump(culture_out, cf, ensure_ascii=False, indent=2)
        cult_df = pd.DataFrame([{'group':'culture', **culture_summary, 'test': culture_test['test'], 'stat': culture_test['stat'], 'p': culture_test['p']}])
        save_csv_with_latex(cult_df, culture_csv)
        summary['saved_files']['culture_json'] = culture_json
        summary['saved_files']['culture_csv'] = culture_csv
        print(f"Saved culture summary JSON and CSV: {culture_json}, {culture_csv}")
    except Exception as e:
        print(f"Failed to save culture summary: {e}")

    # --- Per-question distribution plots for photos and videos ---
    plots_dir = PLOTS_DIR
    os.makedirs(plots_dir, exist_ok=True)

    per_question_summary_rows = []

    def save_violin_for_column(col_name):
        # collect by language
        rows = []
        langs = lang_df['language'].dropna().unique().tolist()
        for lang in langs:
            idx = lang_df[lang_df['language'].str.contains(re.escape(lang), na=False, case=False)].index
            vals = pd.to_numeric(only_study_df.loc[idx, col_name], errors='coerce').dropna()
            if vals.empty:
                continue
            rows.append(pd.DataFrame({'language': lang, 'value': vals}))
            per_question_summary_rows.append({'question': col_name, 'language': lang, 'count': len(vals), 'mean': float(vals.mean()), 'median': float(vals.median()), 'std': float(vals.std(ddof=1) if len(vals)>1 else np.nan), 'min': float(vals.min()), 'max': float(vals.max())})

        if not rows:
            return
        dfplot = pd.concat(rows, ignore_index=True)
        plt.figure(figsize=(6, 4))
        sns.violinplot(x='language', y='value', data=dfplot, inner='quartile', palette='Set2')
        sns.boxplot(x='language', y='value', data=dfplot, showcaps=True, boxprops={'facecolor':'none'}, showfliers=False, whiskerprops={'linewidth':1}, palette='dark')
        plt.title(col_name[:80])
        plt.ylabel('Rating')
        plt.ylim(0.5, 7.5)
        plt.tight_layout()
        safe = re.sub(r'[^0-9A-Za-z._-]+', '_', col_name)[:120]
        out_png = os.path.join(plots_dir, f"violin_{safe}.png")
        plt.savefig(out_png, dpi=150)
        plt.close()

    # process photo columns
    for col in photo_cols:
        if col in only_study_df.columns:
            save_violin_for_column(col)

    # process video columns
    for col in video_cols:
        if col in only_study_df.columns:
            save_violin_for_column(col)

    # save per-question summary CSV
    summary_csv = os.path.join(plots_dir, 'per_question_summary.csv')
    try:
        per_q_df = pd.DataFrame(per_question_summary_rows)
        save_csv_with_latex(per_q_df, summary_csv)
        summary['saved_files']['per_question_summary_csv'] = summary_csv
        print(f"Saved per-question summary CSV: {summary_csv}")
    except Exception as e:
        print(f"Failed to save per-question summary CSV: {e}")

    # update analysis summary JSON
    try:
        with open(json_path, 'w', encoding='utf-8') as jf:
            json.dump(summary, jf, ensure_ascii=False, indent=2)
    except Exception:
        pass

    # --- Produce LaTeX-ready summary tables ---
    def safe_to_latex(df, path, caption=None, label=None):
        try:
            # use pandas to_latex for simplicity
            with open(path, 'w', encoding='utf-8') as f:
                if caption or label:
                    f.write('\\begin{table}[ht]\n')
                    f.write('\\centering\n')
                f.write(df.to_latex(index=False, na_rep='', float_format="%.3f", escape=False))
                if caption:
                    f.write(f"\\caption{{{caption}}}\\n")
                if label:
                    f.write(f"\\label{{{label}}}\\n")
                if caption or label:
                    f.write('\\end{table}\n')
            print(f"Saved LaTeX table: {path}")
            return True
        except Exception as e:
            print(f"Failed to save LaTeX table {path}: {e}")
            return False


    def latexify_df_for_export(df):
        """Return a copy of df with LaTeX-friendly headers and values:
        - convert q_label values to math $q_{i}$
        - rename mean columns to use $\mu$ and mean_diff to $\Delta\mu$
        - escape underscores in textual cells
        """
        df2 = df.copy()
        # Escape underscores in all string cells (except q_label which we will overwrite)
        for col in df2.columns:
            if col == 'q_label':
                continue
            if df2[col].dtype == object:
                df2[col] = df2[col].astype(str).replace({r"_": r"\_"}, regex=True)

        # Convert q_label values like q1 -> $q_{1}$ (formal math)
        if 'q_label' in df2.columns:
            def qlab(x):
                m = re.search(r"(\d+)", str(x))
                return f"$q_{{{m.group(1)}}}$" if m else str(x)
            df2['q_label'] = df2['q_label'].apply(qlab)

        # Build rename map for formal math symbols
        rename_map = {}
        for col in df2.columns:
            # language-specific means: Italiano_mean -> $\mu_{\mathrm{Italiano}}$
            m_mean = re.match(r"^(.*)_(mean)$", col)
            if m_mean:
                lang = m_mean.group(1)
                rename_map[col] = f"$\\mu_{{\\mathrm{{{lang}}}}}$"
                continue

            # sample sizes: _n or n suffix
            m_n = re.match(r"^(.*)_(n)$", col)
            if m_n:
                lang = m_n.group(1)
                rename_map[col] = f"$n_{{\\mathrm{{{lang}}}}}$"
                continue

            # standard deviation columns
            m_std = re.match(r"^(.*)_(std)$", col)
            if m_std:
                lang = m_std.group(1)
                rename_map[col] = f"$s_{{\\mathrm{{{lang}}}}}$"
                continue

            # per-question mean_diff -> Delta mu
            if col == 'mean_diff':
                rename_map[col] = r"$\Delta\mu$"
                continue

            # effect size
            if col in ('cohen_d', 'd'):
                rename_map[col] = r"$d$"
                continue

            # t-test and Mann-Whitney p-values and stats
            if col in ('t_test_p', 't_p', 't_p'):
                rename_map[col] = r"$p_{t}$"
                continue
            if col in ('t_stat',):
                rename_map[col] = r"$t$"
                continue
            if col in ('mannwhitney_p', 'mw_p', 'mw_p'):
                rename_map[col] = r"$p_{\mathrm{MW}}$"
                continue
            if col in ('mannwhitney_u', 'mw_u'):
                rename_map[col] = r"$U$"
                continue

            # adjusted p-values (BH)
            if col.endswith('_p_adj') or col.endswith('_adj'):
                base = col.replace('_p_adj', '').replace('_adj', '')
                if 't' in base:
                    rename_map[col] = r"$p_{t}^{\mathrm{BH}}$"
                elif 'mw' in base or 'mann' in base.lower():
                    rename_map[col] = r"$p_{\mathrm{MW}}^{\mathrm{BH}}$"
                else:
                    rename_map[col] = r"$p^{\mathrm{BH}}$"

        if rename_map:
            df2 = df2.rename(columns=rename_map)

        return df2

    # Group summary table: select concise columns
    try:
        groups_for_table = group_df[[c for c in ['group', 'n_italiano', 'n_deutsch', 'test', 'stat', 'p', 'cohen_d'] if c in group_df.columns]]
    except Exception:
        groups_for_table = group_df.copy()

    # Corresponding-question stats table: pick common diagnostic/test columns
    try:
        # Build a compact per-question table that references questions by label (q1, q2, ...)
        if 'q_label' in comp_df.columns:
            comp_df['q_label'] = comp_df['q_label'].fillna(comp_df['question_index'].apply(lambda x: f"q{int(x)}"))
        else:
            comp_df['q_label'] = comp_df['question_index'].apply(lambda x: f"q{int(x)}")

        stats_cols = [
            'q_label',
            'question_index',
            'mean_diff',
            'chosen_test',
            'chosen_stat',
            'chosen_p',
            'assumptions_met',
        ]
        available_stats_cols = [c for c in stats_cols if c in comp_df.columns]
        comp_for_table = comp_df[available_stats_cols]
    except Exception:
        comp_for_table = comp_df.copy()

    tex_path = os.path.join(LATEX_DIR, 'analysis_tables.tex')
    try:
        with open(tex_path, 'w', encoding='utf-8') as tf:
            tf.write('% Analysis tables generated by analyze.py\n')
        # Save group and corresponding-question tables (existing behavior)
        groups_tex = os.path.join(LATEX_DIR, 'groups_table.tex')
        questions_tex = os.path.join(LATEX_DIR, 'questions_stats_table.tex')
        safe_to_latex(latexify_df_for_export(groups_for_table), groups_tex, caption='Group comparisons (means and tests)', label='tab:groups')
        safe_to_latex(latexify_df_for_export(comp_for_table.head(50)), questions_tex, caption='Per-question statistical results (first 50 rows)', label='tab:questions')
        summary['saved_files']['latex_tables'] = {
            'groups_table': groups_tex,
            'questions_table': questions_tex,
        }

        # --- Export additional results as separate LaTeX tables ---
        # Per-language stats (one table per language)
        try:
            for lang, dfstat in stats_by_lang.items():
                if isinstance(dfstat, pd.DataFrame) and not dfstat.empty:
                    statdf = dfstat.reset_index().rename(columns={'index': 'metric'})
                    # rename mean -> \mu in the per-language stat table
                    statdf = statdf.rename(columns={c: (r"$\\mu$" if c == 'mean' else c) for c in statdf.columns})
                    safe_to_latex(latexify_df_for_export(statdf),
                                  os.path.join(LATEX_DIR, f"stats_by_lang_{re.sub(r'[^0-9A-Za-z]+','_', lang) }.tex"),
                                  caption=f'Summary statistics for {lang}',
                                  label=f'tab:stats_{re.sub(r"[^0-9A-Za-z]+","_", lang)}')
        except Exception:
            pass

        # Question index LaTeX (export the q-label mapping if available)
        try:
                if 'questions_index_df' in locals() and not questions_index_df.empty:
                    qtex = os.path.join(LATEX_DIR, 'questions_index.tex')
                    qdf = questions_index_df[["q_label", "Italiano_col", "Deutsch_col"]].rename(columns={"Italiano_col":"Italiano", "Deutsch_col":"Deutsch"})
                    safe_to_latex(latexify_df_for_export(qdf), qtex, caption='Question index (use q_i in other tables)', label='tab:questions_index')
                    summary['saved_files']['questions_index_tex'] = qtex
        except Exception:
            pass

        # Full corresponding-question table
        try:
                if not comp_df.empty:
                    safe_to_latex(latexify_df_for_export(comp_df), os.path.join(LATEX_DIR, 'corresponding_question_comparison_full.tex'),
                                  caption='Full corresponding question diagnostics and test results',
                                  label='tab:corresponding_full')
                    summary['saved_files']['corresponding_question_comparison_full'] = os.path.join(LATEX_DIR, 'corresponding_question_comparison_full.tex')
        except Exception:
            pass

        # Per-language comparisons flat
        try:
            per_lang_rows = []
            if 'rows' in locals():
                per_lang_rows = rows
            per_lang_df = pd.DataFrame(per_lang_rows) if per_lang_rows else None
            if per_lang_df is not None and not per_lang_df.empty:
                safe_to_latex(latexify_df_for_export(per_lang_df), os.path.join(LATEX_DIR, 'per_language_comparisons_flat.tex'),
                              caption='Per-language comparison results (flat)', label='tab:perlang')
                summary['saved_files']['per_language_comparisons_flat_tex'] = os.path.join(LATEX_DIR, 'per_language_comparisons_flat.tex')
        except Exception:
            pass

        # Group comparisons (auto)
        try:
            if not group_df.empty:
                safe_to_latex(latexify_df_for_export(group_df), os.path.join(LATEX_DIR, 'group_comparisons_auto.tex'),
                              caption='Automatic group comparisons', label='tab:groups_auto')
                summary['saved_files']['group_comparisons_auto_tex'] = os.path.join(LATEX_DIR, 'group_comparisons_auto.tex')
        except Exception:
            pass

        # Culture summary
        try:
            cult_df = pd.DataFrame([{'group':'culture', **culture_summary, 'test': culture_test['test'], 'stat': culture_test['stat'], 'p': culture_test['p']}])
            safe_to_latex(latexify_df_for_export(cult_df), os.path.join(LATEX_DIR, 'culture_summary.tex'),
                          caption='Culture group summary and test', label='tab:culture')
            summary['saved_files']['culture_tex'] = os.path.join(LATEX_DIR, 'culture_summary.tex')
        except Exception:
            pass

        # Per-question summary (from plots)
        try:
            per_q_path = os.path.join(plots_dir, 'per_question_summary.csv')
            if os.path.exists(per_q_path):
                per_q_df = pd.read_csv(per_q_path)
                if not per_q_df.empty:
                    safe_to_latex(latexify_df_for_export(per_q_df), os.path.join(LATEX_DIR, 'per_question_summary.tex'),
                                  caption='Per-question summary (counts, mean, median, std)', label='tab:perq')
                    summary['saved_files']['per_question_summary_tex'] = os.path.join(LATEX_DIR, 'per_question_summary.tex')
        except Exception:
            pass

        # Significant results (robust / parametric) if present
        try:
            if 'significant_robust_df' in locals() and not significant_robust_df.empty:
                safe_to_latex(latexify_df_for_export(significant_robust_df), os.path.join(LATEX_DIR, 'significant_corresponding_comparisons_robust.tex'),
                              caption='Significant robust corresponding comparisons', label='tab:sig_robust')
                summary['saved_files']['significant_robust_tex'] = os.path.join(LATEX_DIR, 'significant_corresponding_comparisons_robust.tex')
        except Exception:
            pass

        try:
            if 'sign_rows_ttest' in locals() and sign_rows_ttest:
                sig_t_df = pd.DataFrame(sign_rows_ttest)
                if not sig_t_df.empty:
                        safe_to_latex(latexify_df_for_export(sig_t_df), os.path.join(LATEX_DIR, 'significant_corresponding_comparisons_parametric.tex'),
                                      caption='Significant parametric corresponding comparisons', label='tab:sig_param')
                        summary['saved_files']['significant_parametric_tex'] = os.path.join(LATEX_DIR, 'significant_corresponding_comparisons_parametric.tex')
        except Exception:
            pass

        # append to main tex file by including the saved files
        with open(tex_path, 'a', encoding='utf-8') as tf:
            tf.write('\\input{groups_table.tex}\n')
            tf.write('\\input{questions_stats_table.tex}\n')
            tf.write('\\input{group_comparisons_auto.tex}\n')
            tf.write('\\input{corresponding_question_comparison_full.tex}\n')
            tf.write('\\input{per_language_comparisons_flat.tex}\n')
            tf.write('\\input{culture_summary.tex}\n')
            tf.write('\\input{per_question_summary.tex}\n')
        print(f"Saved combined LaTeX wrapper: {tex_path}")
        with open(json_path, 'w', encoding='utf-8') as jf:
            json.dump(summary, jf, ensure_ascii=False, indent=2)
    except Exception as e:
        print(f"Failed to produce LaTeX tables: {e}")


if __name__ == "__main__":
    main()
