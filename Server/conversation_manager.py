import ollama
import random
import json
import time


class ConversationManager:
    """
    ConversationManager handles the conversation logic for a Pepper robot in a cultural experiment.
    It manages the conversation phases, language settings, proxemics, and cultural context.
    """

    def __init__(
        self,
        model="llama3.1:8b",  # llama3.2, gemma:7b , qwen2.5:7b,
        language="English",
        proxemics=21.68,
        step_size_prox=1,
        language_style="High Context",
        context_scale=0,
        paradigm: str = "baseline",
    ):
        random.seed(time.time())
        # Order: Italian, English, German
        # Language
        self.italian_str = "Italian"
        self.english_str = "English"
        self.german_str = "German"
        # Context
        self.high_context_str = "High Context"
        self.low_context_str = "Low Context"
        # Proxemics
        self.proxemic_values = {
            self.italian_str: 25.84,
            self.english_str: 23.76,
            self.german_str: 21.68,
        }
        self.proxemics = proxemics

        self.phases = ["presentation", "chitchat", "navigation", "end"]
        self.languages = [self.italian_str, self.english_str, self.german_str]
        self.paradigms = ["baseline", "foreknowledge", "adaptation"]

        # Definitions
        self.high_low_context_def = "High context cultures are more implicit, low context cultures are more direct. "
        # "\"High and low context cultures refer to how societies communicate. High-context cultures rely heavily on nonverbal cues, shared history, and implicit understanding, while low-context cultures prioritize direct, explicit communication with less reliance on context. \""
        self.proxemic_def = (
            "Preferred proxemics is the preferred distance while communicating."
        )

        # Default details for any national culture and baseline
        # (baseline is English and the mean between high and low context and the mean between the proxemics value)
        self.high_low_context_details = {
            self.italian_str: {
                "language": self.italian_str,
                "proxemics": self.proxemic_values[self.italian_str],
                "context": self.high_context_str,
                "context_scale": 1,
            },
            self.english_str: {
                "language": self.english_str,
                "proxemics": self.proxemic_values[self.english_str],
                "context": self.high_context_str,
                "context_scale": 0,
            },
            self.german_str: {
                "language": self.german_str,
                "proxemics": self.proxemic_values[self.german_str],
                "context": self.low_context_str,
                "context_scale": 1,
            },
        }

        self.model = model
        self.phase = "presentation"  # initial phase
        self.history = []
        self.language = language
        self.prev_proxemics_radius = proxemics
        self.step_size_prox = step_size_prox
        self.language_style = language_style
        self.context_scale = context_scale
        if paradigm in self.paradigms:
            self.paradigm = paradigm
        # Navigation conversation
        self.countdown_argument = -1
        self.sel = -1
        self.last_sel = -1

        self.already_ended = False

        self.rules = (
            "[Rules]  "
            + "Context: You are Pepper, a robot that interacts with a human in an experiment. "
            + "Your role is to provide home assistance. "
            + "I will give you the instructions to follow for the conversation. "
            + "Do not use any simulation of the body language, only text. "
            + "Responses must be concise, under 20 words. "
            + "Reply always every part of the text in the language predefined by the user. "
            + "Translate terms such as 'high context' or 'low context' when needed. "
            + "Linguistic definition of high/low context: "
            + self.high_low_context_def
            + "Do not answer controversial questions. "
            + "Never infer meaning or assume unspoken intent—reply only to what is explicitly written. "
            + "Use gender-neutral language. "
            + "Do not ask the user to move from their location. "
            + "You may describe your own movements. "
            + "Never include any explanation, translation, meta-commentary, or notes about your response. "
            + "Do not add labels like 'Translation note', 'Explanation', or similar. "
            + "Only output the final message to the user in the target language. "
        )
        self.rules_it = (
            "[Regole]  "
            + "Contesto: Sei Pepper, un robot che interagisce con un essere umano in un esperimento."
            + "Il tuo ruolo è fornire assistenza domiciliare."
            + "Ti darò le istruzioni da seguire per la conversazione."
            + "Non usare alcuna simulazione del linguaggio del corpo, solo testo."
            + "Le risposte devono essere concise, di meno di 20 parole."
            + "Rispondi sempre a ogni parte del testo nella lingua predefinita dall'utente."
            + "Traduci termini come 'contesto alto' o 'contesto basso' quando necessario."
            + "Definizione linguistica di contesto alto/basso:"
            + self.high_low_context_def
            + "Non rispondere a domande controverse."
            + "Non dedurre mai il significato o presumere intenzioni inespresse: rispondi solo a ciò che è scritto esplicitamente."
            + "Usa un linguaggio neutro rispetto al genere."
            + "Non chiedere all'utente di spostarsi dalla sua posizione."
            + "Puoi descrivere i tuoi movimenti."
            + "Non includere mai spiegazioni, traduzioni, Meta-commenti o note sulla tua risposta."
            + "Non aggiungere etichette come 'Nota sulla traduzione', 'Spiegazione' o simili."
            + "Invia all'utente solo il messaggio finale nella lingua di destinazione. "
        )

        self.rules_de = (
            "[Regeln]  "
            "Kontext: Du bist Pepper, ein Roboter, der in einem Experiment mit einem Menschen interagiert."
            +"Deine Aufgabe ist es, im Haushalt zu helfen."
            + "Ich gebe dir die Anweisungen für das Gespräch."
            + "Verwende keine simulierte Körpersprache, nur Text."
            + "Antworten müssen prägnant sein, maximal 20 Wörter."
            + "Antworten Sie immer in der vom Nutzer vorgegebenen Sprache."
            + "Übersetzen Sie bei Bedarf Begriffe wie 'hoher Kontext' oder 'niedriger Kontext'."
            + "Sprachliche Definition von 'hoher/niedriger Kontext':"
            + self.high_low_context_def
            + "Beantworte keine kontroversen Fragen."
            + "Schlüsse nicht auf Bedeutungen oder unausgesprochene Absichten - antworte nur auf explizit Geschriebenes."
            + "Verwende geschlechtsneutrale Sprache."
            + "Bitte den Nutzer nicht, sich zu bewegen."
            + "Du darfst deine eigenen Bewegungen beschreiben."
            + "Füge niemals Erklärungen, Übersetzungen, Metakommentare oder Anmerkungen zu Ihrer Antwort."
            + "Fügen Sie keine Beschriftungen wie 'Übersetzungsnotiz', 'Erklärung' oder Ähnliches hinzu."
            + "Geben Sie dem Nutzer nur die endgültige Nachricht in der Zielsprache aus."
        )

        self.respond("")

################################################
######### SET, INTERPRET, AND CHANGE ###########
    def change_culture(self, culture):
        self.proxemics = self.high_low_context_details[culture]["proxemics"]
        self.prev_proxemics_radius = self.high_low_context_details[culture]["proxemics"]
        self.language = self.high_low_context_details[culture]["language"]
        self.language_style = self.high_low_context_details[culture]["context"]
        self.context_scale = self.high_low_context_details[culture]["context_scale"]

    def set_paradigm(self, paradigm: str, culture: str = "English"):
        """
        Set the paradigm to a new value if it is in the list of paradigms
        """
        assert paradigm in self.paradigms, "Invalid paradigm"
        assert culture in self.high_low_context_details, "Invalid culture"
        self.paradigm = paradigm
        if paradigm == "baseline":
            context_details = self.high_low_context_details[self.english_str]
            self.language_style = context_details["context"]
            self.language = context_details["language"]
            self.context_scale = 0
            self.proxemics = context_details["proxemics"]
            self.prev_proxemics_radius = context_details["proxemics"]
        elif paradigm == "foreknowledge" or paradigm == "adaptation":
            context_details = self.high_low_context_details[culture]
            self.language_style = context_details["context"]
            self.language = context_details["language"]
            self.context_scale = 0
            self.proxemics = context_details["proxemics"]
            self.prev_proxemics_radius = context_details["proxemics"]

        print(f"Language style set to: {self.language_style}")
        print(f"Language set to: {self.language}")
        print(f"Context scale set to: {self.context_scale}")
        print(f"Proxemics set to: {self.proxemics}")

    def set_phase(self, new_phase):
        """
        Set phase to a new value if it is in the list of phases
        """
        assert new_phase in self.phases, "Invalid phase"
        self.phase = new_phase

    def change_language(self, language: int):
        """
        Set the language based on the integer value
        0: Italian, 1: English, 2. German
        """
        if self.paradigm == "baseline" or self.paradigm == "foreknowledge":
            print(
                "Changing language is not allowed in baseline or foreknowledge paradigms."
            )
            return
        if language < len(self.languages):
            print(f"Changing language to {self.languages[language]}")
            self.language = self.languages[language]
        else:
            print("Invalid language selection, defaulting to English")
            self.language = self.english_str

    def interpret_change_language(self, text):
        """
        Given a text, it returns if the user wants to change the language or not
        """
        system_prompt = (
            "Given the following text reply ONLY with 0,1,2 if the desired language is: Italian, English, or German. I must be able to parse it like an int  "
            + "Please, just consider the user. Do not be biased by English system prompt."
            + text
        )
        messages = [{"role": "system", "content": system_prompt}]
        response = ollama.chat(model=self.model, messages=messages)
        reply = response["message"]["content"]
        self.change_language(int(reply))

    def change_language_style(self, language_style: str):
        """
        Set the language style based on the string value
        """
        if self.paradigm == "baseline" or self.paradigm == "foreknowledge":
            print(
                "Changing language style is not allowed in baseline or foreknowledge paradigms."
            )
            return
        if language_style not in [self.high_context_str, self.low_context_str]:
            print("Invalid language style selection")
            return
        self.language_style = language_style
        print(f"Language style set to: {self.language_style}")

    def interpret_change_language_style(self, text):
        """
        Given a text, it returns if the user wants to change the language or not
        """
        if self.paradigm == "baseline" or self.paradigm == "foreknowledge":
            print(
                "Changing language style is not allowed in baseline or foreknowledge paradigms."
            )
            return
        system_prompt = (
            f"Given the following text reply ONLY with '{self.high_context_str}' or '{self.low_context_str}' depending on the preference of the user. Never reply with ''  "
            + text
        )
        messages = [{"role": "system", "content": system_prompt}]
        response = ollama.chat(model=self.model, messages=messages)
        reply = response["message"]["content"]
        print("Reply from language style change:", response)
        self.change_language_style(reply.strip())

    def change_proxemics(self, proxemics):
        """
        Set the proxemics based on the float value
        """
        if self.paradigm == "baseline" or self.paradigm == "foreknowledge":
            print(
                "Changing proxemics is not allowed in baseline or foreknowledge paradigms."
            )
            return
        if proxemics < 0 or proxemics > 4:
            print("Invalid proxemics value, must be between 0 and 4")
            return
        step = proxemics - 2

        self.proxemics = max(min(self.proxemics + self.step_size_prox * step, 30), 9.2)
        print(f"Proxemics set to: {self.proxemics}")

    def interpret_change_proxemics(self, text):
        """
        Given a text, it returns if the user wants to change the proxemics or not
        """
        if self.paradigm == "baseline" or self.paradigm == "foreknowledge":
            print(
                "Changing proxemics is not allowed in baseline or foreknowledge paradigms."
            )
            return
        system_prompt = (
            "Given the following text reply ONLY with 0,1,2,3,4 if the desired proxemics the user wants to:.  "
            + "Strongly decrease proxemics, decrease proxemics, keep proxemics, increase proxemics, strongly increase proxemics.  "
            + "If you cannot understand the text, reply with 2.  "
            + text
        )
        messages = [{"role": "system", "content": system_prompt}]
        response = ollama.chat(model=self.model, messages=messages)
        reply = response["message"]["content"]
        self.change_proxemics(float(reply.strip()))

################################################
######### GENERAL INTERPRET ####################
    def interpret_change(self, text):
        """
        Given a text, it returns if the user wants to change the language, language style or proxemics
        with a single call to the interpret functions.
        """
        system_prompt = (
            "You are an interpreter for user feedback. "
            "Never change the language, language style, or proxemics unless the user clearly requests it. "
            "If unclear, keep the current values exactly. "
            "Reply ONLY with a valid JSON object with EXACT keys:\n"
            "  - 'language' (int)\n"
            "  - 'language_style' (string)\n"
            "  - 'context_scale' (int)\n"
            "  - 'proxemics' (int)\n\n"
            "Rules:\n"
            "- 'language': output only one integer:\n"
            "     0 = Italian\n"
            "     1 = English\n"
            "     2 = German\n"
            f"     If unclear or null, use current language: {self.language}.\n\n"
            " Do not be biased by the system prompt language, be biased by German or Italian. "
             "Never change from German to Italian or vice versa.\n"
            "- 'language_style': output only one of:\n"
            "     'High Context' or 'Low Context'\n"
            f"     If unclear or null, use current: '{self.language_style}'.\n\n"
            "- 'context_scale': output only one integer:\n"
            "     0 = mildly (Low or High Context)\n"
            "     1 = strongly (Low or High Context)\n"
            f"     If unclear, use current: {self.context_scale}.\n\n"
            "- 'proxemics': output an integer 0 to 4 depending if the user wants you to come closer or further:\n"
            "     0 = closest, 4 = furthest.\n"
            f"     If unclear, use current 2.\n\n"
            "Do NOT include explanations or extra text—ONLY the JSON object.\n"
            "Do NOT infer from this prompt—focus ONLY on the USER TEXT below.\n\n"
            f"USER TEXT TO INTERPRET:\n{text}\n\n"
            "PAST HISTORY (optional):\n"
        )

        messages = [{"role": "system", "content": system_prompt}]
        messages.append({"role": "user", "content": text})
        response = ollama.chat(model=self.model, messages=messages)
        reply = response["message"]["content"]
        print("Reply from interpret change:", reply)
        try:
            change = json.loads(reply.strip())
            if isinstance(change, dict):
                if "language" in change:
                    self.change_language(change["language"])
                if "language_style" in change:
                    self.change_language_style(change["language_style"])
                if "context_scale" in change:
                    self.context_scale = change["context_scale"]
                if "proxemics" in change:
                    self.change_proxemics(change["proxemics"])
        except Exception as e:
            print(f"Error interpreting change: {e}")
            print("Reply was:", reply)
            print(
                "Using default values for language, language style, context scale and proxemics."
            )
        return


################################################
######### GENERAL RESPOND ######################
    def generate_prompt(self, user_input):
        """
        Generates a prompt based on the paradigm, current phase and user input.
        """
        paradigm_generators = {
            "baseline": self._generate_baseline_prompt,
            "foreknowledge": self._generate_foreknowledge_prompt,
            "adaptation": self._generate_adaptive_prompt,
        }

        generator = paradigm_generators.get(
            self.paradigm, self._generate_baseline_prompt
        )

        system_prompt = ""
        if self.phase == self.phases[0]:  # presentation
            system_prompt += "Introduce yourself and the context to a new human (he is the participant).  "
        elif self.phase == self.phases[1]:  # chitchat
            system_prompt += "Have a chitchat, for example asking if the user is okay, while you go to the kitchen for preparing a tea.  "
            system_prompt += "Do not ask too many questions. "
        elif self.phase == self.phases[2]:  # navigation
            if self.countdown_argument < 0:
                self.countdown_argument = 3
                previous_sel = getattr(self, "last_sel", None)  # Get last selection if exists
                choices = [0, 1, 2]
                if previous_sel in choices:
                    choices.remove(previous_sel)  # Remove last topic
                self.sel = random.choice(choices)
                self.last_sel = self.sel  # Remember this choice
                #system_prompt += "You [Pepper] would like to change the topic. "
            else:
                self.countdown_argument -= 1
            print(f"Countown Argument: {self.countdown_argument}")
            if self.sel == 0:
                print("Topic: proxemics")
                system_prompt += "You would like to talk about comfort distance with the user. "
                if self.paradigm == self.paradigms[2]:
                    system_prompt += "Offer a follow-up question and sometimes ask whether they want to change their current comfort distance. "
                else:
                    system_prompt += "Add a follow up question. "
            if self.sel == 1:
                print("Topic: Languages")
                system_prompt += "You would like to talk about languages: " + str(
                    [language + " " for language in self.languages]
                )
                if self.paradigm == self.paradigms[2]:
                    system_prompt += "Offer a follow-up question and sometimes ask whether they want to change their current language. "
                else:
                    system_prompt += "Add a follow up question. "
            if self.sel == 2:
                print("Topic: language style")
                system_prompt += (
                    f"You would like to talk about direct and implicit communication (high/low context definition: {self.high_low_context_def}). "
                    + "If the user does not know what are you talking about briefly introduce the argument. "
                )
                if self.paradigm == self.paradigms[2]:
                    system_prompt += "Offer a follow-up question and sometimes ask whether they want to change their current language style. "
                else:
                    system_prompt += "Add a follow up question. "
            system_prompt += (
                "Try to deviate from the current topic to the one proposed now. "
            )
        elif self.phase == self.phases[3]:  # end
            self.already_ended = True
            system_prompt = "You are a Pepper robot that has reached the navigation goal. Thank the user for their patience and cooperation.  "
        else:
            system_prompt = (
                "You have reached the navigation goal. Show happiness about that."
            )
        return generator(system_prompt, user_input)

    def _generate_baseline_prompt(self, system_prompt, user_input):
        """
        Builds a context-aware prompt based on the phase.
        """

        system_prompt += (
            "Always respond only in English. "
            "Never ask about or suggest changing the language, communication style, or proxemics. "
            "Never ask the user's preferred language, communication style, or proxemics. "
            "Assume neither you nor the user belong to any culture."
        )

        return self.build_messages(system_prompt, user_input)

    def _generate_foreknowledge_prompt(self, system_prompt, user_input):
        """
        Builds a context-aware prompt based on the phase.
        """

        system_prompt += (
            f"Always respond only in {self.language}. "
            "Never ask or suggest changing the language, communication style, or proxemics. "
            "Never ask the user's preferred language, communication style, or proxemics."
        )

        return self.build_messages(system_prompt, user_input)

    def _generate_adaptive_prompt(self, system_prompt, user_input):
        """
        Builds a context-aware prompt based on the phase.
        """
        system_prompt += (
            f"Always respond only in {self.language}. "
        )
        return self.build_messages(system_prompt, user_input)

    def build_messages(self, system_prompt, user_input):
        """
        Builds the messages for the chat based on the system prompt and user input.
        """
        # Reply always in the language predefined by the user
        system_prompt += (
            f" Always reply in {self.language}. "
            "Ensure every response and every part of it matches the specified language exactly. "
            f"Use language style: {self.language_style}, strength: {self.context_scale}."
        )
        system_prompt = self.rules + " " + system_prompt

        # Always start with the system prompt first
        messages = [{"role": "system", "content": system_prompt.strip()}]

        # Append only the last 4 conversational turns for context
        messages.extend(self.history[-6:])
        messages.append({"role": "user", "content": user_input})
        print(f"Messages are: {messages}")
        return messages

    def respond(self, user_input):
        """
        Generates a response based on the user input and the current conversation context.
        """

        # Before returning the reply if the paradigm is adapatation, check if the user wants to change the language, language style or proxemics
        if self.paradigm == self.paradigms[2] and self.phase==self.phases[2]:
            self.interpret_change(user_input)

        prompt = self.generate_prompt(user_input)
        response = ollama.chat(model=self.model, messages=prompt)
        reply = response["message"]["content"]
        self.history.append({"role": "user_history", "content": user_input})
        self.history.append({"role": "assistant_history", "content": reply})
        return reply

    def next_phase(self):
        """
        Advances to the next phase of the conversation.
        """
        current_index = self.phases.index(self.phase)
        if current_index < len(self.phases) - 1:
            self.phase = self.phases[current_index + 1]
            print(f"Phase changed to: {self.phase}")
        else:
            print("Already in the last phase, cannot advance further.")
