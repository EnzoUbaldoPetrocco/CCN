import ollama

def read_phrases(file_path):
    """Read phrases from a text file and return as a single string."""
    try:
        with open(file_path, 'r', encoding='utf-8') as file:
            phrases = ' '.join(f"\nAnswer: {i}: {line.strip()}" for i, line in enumerate(file) if line.strip())
        return phrases
    except FileNotFoundError:
        print(f"Error: File {file_path} not found.")
        return ""

def analyze_phrase(phrases, model="qwen3-vl:8b"):
    """Analyze a single phrase using Ollama."""
    try:
        response = ollama.generate(
            model=model,
            prompt=f"I have gathered the answers of a HRI with Pepper robot. The question is the following: Today, the robot interacted with you in three different ways: 1. Neutral: The robot used English and general behaviors. 2. National Knowledge: The robot used your native language and communication style. 3. Adaptive: The robot adjusted its interactions to your personal preferences. Thinking about your experience: Did your answers to the questionnaire change between the first (Neutral) and second (National Knowledge) interactions? If so, why do you think that happened? Note that the order in which they are listed here does not reflect the chronological order in which they interacted with. I need you to analyze those answers:\n\n{chr(10).join(phrases)}",
            stream=False
        )
        return response['response']
    except Exception as e:
        print(f"Error analyzing phrase: {e}")
        return None

def main():
    phrases_file = "phrases.txt"
    phrases = read_phrases(phrases_file)
    
    if not phrases:
        print("No phrases found to analyze.")
        return
    
    print(f"Original: {phrases}")
    print(f"Analysis:")
    analysis = analyze_phrase(phrases)
    if analysis:
        print(analysis)
    print()

if __name__ == "__main__":
    main()