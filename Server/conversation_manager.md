# conversation_manager.py

## Purpose
Manages conversational interactions for a Pepper robot, handling multiple languages (Italian, English, German), cultural contexts (high/low context communication), and conversation phases.

## Classes

### `ConversationManager`
Main class for managing robot conversation logic.

**Key Attributes:**
- `model`: LLM model (default: llama3.1:8b)
- `language`: Current language (Italian, English, German)
- `proxemics`: Preferred interaction distance
- `paradigm`: Experiment paradigm (baseline, foreknowledge, adaptation)
- `phase`: Conversation phase (presentation, chitchat, navigation, end)
- `language_style`: Communication style (High Context, Low Context)

**Key Methods:**
- `__init__()`: Initialize conversation manager with language, proxemics, and paradigm settings
- `set_paradigm(paradigm, culture)`: Set the experimental paradigm
- `set_phase(new_phase)`: Change conversation phase
- `change_culture(culture)`: Update language and proxemics based on cultural settings
- `change_language(language)`: Switch between Italian/English/German
- `interpret_change_language(text)`: Use LLM to detect language preference from user text
- `change_language_style(style)`: Switch between high and low context communication
- `interpret_change_language_style(text)`: Use LLM to detect communication style preference
- `change_proxemics(proxemics)`: Adjust preferred interaction distance
- `interpret_change_proxemics(text)`: Use LLM to detect proxemics preference
- `interpret_change(text)`: Single LLM call to detect all preference changes (language, style, proxemics)
- `generate_prompt(user_input)`: Create context-aware prompts based on paradigm and phase
- `_generate_baseline_prompt()`: Generate English-only baseline prompts
- `_generate_foreknowledge_prompt()`: Generate language-fixed foreknowledge prompts
- `_generate_adaptive_prompt()`: Generate adaptive prompts allowing preference changes
- `build_messages()`: Construct chat messages with system rules and history
- `respond(user_input)`: Get LLM response via Ollama
- `next_phase()`: Advance to next conversation phase

## Technical Details

### Cultural Adaptation Model
The system implements a three-dimensional cultural adaptation framework:

1. **Language Selection**: Italian (High Context, Proxemics: 25.84m), English (Low Context, Proxemics: 23.76m), German (Low Context, Proxemics: 21.68m)

2. **Communication Style**: 
   - High Context: Implicit, indirect communication relying on shared understanding
   - Low Context: Explicit, direct communication with minimal assumptions

3. **Proxemics**: Preferred interpersonal distance based on Hall's proxemics theory, adapted for robot-human interaction

### LLM Integration
- **Model**: Llama 3.1 8B via Ollama API
- **Prompt Engineering**: Context-aware prompts that incorporate cultural parameters and conversation phase
- **Multi-language Support**: System prompts translated to Italian, English, and German
- **Response Constraints**: Maximum 20 words per response for natural interaction pacing

### Paradigm Implementation
- **Baseline**: Fixed English language, neutral context scale (0), standard proxemics
- **Foreknowledge**: Pre-assigned cultural parameters, no runtime adaptation
- **Adaptation**: Runtime preference detection and dynamic parameter adjustment

### Preference Detection Algorithm
Uses LLM-based natural language understanding to detect user preferences from conversational input:

```python
def interpret_change_language_style(self, text):
    system_prompt = f"Analyze: '{text}'. Respond with 'high' or 'low' for communication style preference."
    response = ollama.chat(model=self.model, messages=[{"role": "system", "content": system_prompt}])
    return response["message"]["content"].strip().lower()
```

### Conversation Flow Control
Implements a finite state machine with phases:
- **Presentation**: Initial greeting and setup
- **Chitchat**: Casual conversation to build rapport
- **Navigation**: Task-oriented interaction during movement
- **End**: Conversation closure

### Technical Challenges
- **Cultural Sensitivity**: Balancing authentic cultural representation with ethical AI guidelines
- **Real-time Adaptation**: Maintaining conversation coherence during parameter changes
- **Multilingual Consistency**: Ensuring equivalent expressiveness across languages
- **Proxemics Integration**: Coordinating verbal communication with physical distance preferences</content>
<parameter name="filePath">c:\Users\Utente\Desktop\CCN\Server\conversation_manager.md