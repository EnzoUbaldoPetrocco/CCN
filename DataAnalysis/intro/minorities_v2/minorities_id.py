import pandas as pd
import numpy as np
import seaborn as sns
import matplotlib.pyplot as plt

label_map = {
    "Participant ID": "id",
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
  

def parse_intro_info(df):
    df = df.drop(columns=["Informazioni cronologiche"])
    df = df.replace('Disagree strongly', 1)
    df = df.replace('Disagree a little', 2)
    df = df.replace('Neither agree or disagree', 3)
    df = df.replace('Agree a little', 4)
    df = df.replace('Agree strongly', 5)

    def percent_to_float(x):
        if isinstance(x, str) and x.strip().endswith("%"):
            try:
                return float(x.strip().replace("%", "")) / 100
            except ValueError:
                return np.nan
        return x
    # Apply everywhere
    df = df.map(percent_to_float)

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
    
    df = df.rename(columns=label_map)

    df = df[["id","mean_cultural_closeness_subj", "mean_personality_subj", "extraversion", "agreebleness", "coscientiousness", "neuroticism", "openness", "mean_trust_subj"]]

    cols_to_normalize = df.columns[1:]
    df[cols_to_normalize] = df[cols_to_normalize].apply(lambda x: (x - x.min()) / (x.max() - x.min()))
    
    print(df.head())
    
    return df


def identify_outliers(df):
    for magic_number in [0.8, 1.0, 1.5, 3]:
        all_lower_list = []
        all_upper_list = []
        for col in df.columns[1:]:
            Q1 = df[col].quantile(0.25)
            Q3 = df[col].quantile(0.75)
            IQR = Q3 - Q1
            
            lower_bound = Q1 - magic_number * IQR
            upper_bound = Q3 + magic_number * IQR
            
            # 2. Append the actual dataframe slices to our lists
            # We add a column to remember WHICH variable triggered the outlier status
            lows = df[df[col] < lower_bound].copy()
            lows['outlier_source'] = col
            all_lower_list.append(lows)
            
            highs = df[df[col] > upper_bound].copy()
            highs['outlier_source'] = col
            all_upper_list.append(highs)

        final_lower = pd.concat(all_lower_list).drop_duplicates()
        final_upper = pd.concat(all_upper_list).drop_duplicates()

        final_lower.to_json(f"{magic_number}_lower_outliers.json", orient="records", indent=4)
        final_upper.to_json(f"{magic_number}_upper_outliers.json", orient="records", indent=4)

    


def main():
    file_path = "../Intro CCN  (Risposte).CSV"
    df = pd.read_csv(file_path)
    print(df.head())
    df = parse_intro_info(df)
    identify_outliers(df)


if __name__ == "__main__":
    main()
    