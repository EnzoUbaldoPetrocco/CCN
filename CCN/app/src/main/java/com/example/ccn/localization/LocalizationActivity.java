package com.example.ccn.localization;

import android.Manifest;
import android.content.Intent;
import android.content.pm.PackageManager;
import android.os.Bundle;
import android.os.Handler;
import android.os.Looper;
import android.speech.RecognitionListener;
import android.speech.RecognizerIntent;
import android.speech.SpeechRecognizer;
import android.speech.tts.TextToSpeech;
import android.speech.tts.UtteranceProgressListener;
import android.speech.tts.Voice;
import android.util.Log;
import android.widget.ImageView;
import android.widget.TextView;
import android.widget.Toast;

import com.aldebaran.qi.Future;
import com.aldebaran.qi.sdk.Qi;
import com.aldebaran.qi.sdk.QiContext;
import com.aldebaran.qi.sdk.QiSDK;
import com.aldebaran.qi.sdk.RobotLifecycleCallbacks;
import com.aldebaran.qi.sdk.design.activity.RobotActivity;
import com.aldebaran.qi.sdk.object.autonomousabilities.AutonomousAbilities;
import com.aldebaran.qi.sdk.object.autonomousabilities.AutonomousAbilityHolder;
import com.aldebaran.qi.sdk.object.context.RobotContext;
import com.aldebaran.qi.sdk.util.FutureUtils;
import com.example.ccn.R;
import com.example.ccn.core.ClientManager;
import com.example.ccn.localization.adaptation.GoToGoalAdaptationScreen;
import com.example.ccn.localization.baseline.GoToGoalBaselineScreen;
import com.example.ccn.localization.foreknowledge.GoToGoalForeknowledgeScreen;
import com.example.ccn.localization.gotogoal.GoToGoalScreen;
import com.example.ccn.localization.localizationmenu.LocalizationMenuScreen;
import com.example.ccn.localization.localize.LocalizeScreen;
import com.example.ccn.mapping.MappingEvent;

import androidx.annotation.NonNull;
import androidx.annotation.Nullable;
import androidx.annotation.StringRes;
import androidx.appcompat.app.AppCompatActivity;
import androidx.core.app.ActivityCompat;
import androidx.core.content.ContextCompat;
import androidx.fragment.app.Fragment;

import org.json.JSONArray;

import java.util.ArrayList;
import java.util.Locale;
import java.util.Set;
import java.util.concurrent.Executors;
import java.util.concurrent.ScheduledExecutorService;
import java.util.concurrent.TimeUnit;

import butterknife.BindView;
import butterknife.ButterKnife;
import butterknife.OnClick;
import io.reactivex.disposables.Disposable;
import io.reactivex.schedulers.Schedulers;
import kotlin.Unit;
import kotlin.jvm.functions.Function1;


/**
 * The localization activity
 */
public class LocalizationActivity extends AppCompatActivity implements  RobotLifecycleCallbacks {

    @NonNull
    private static final String TAG = "LocalizationActivity";
    @NonNull
    private final ScreenMachine screenMachine = new ScreenMachine();
    @NonNull
    private final LocalizeManager localizeManager= new LocalizeManager();
    @Nullable
    private QiContext qiContext;
    @Nullable
    private Screen currentScreen;
    @Nullable
    private Disposable disposable;

    TextView titleTextView;

    private ScheduledExecutorService scheduler;
    private final Handler mainHandler = new Handler(Looper.getMainLooper());
    AutonomousAbilityHolder holderBack;
    AutonomousAbilityHolder holderBasic;

    // Speaking abilities
    private final int REQUEST_CODE_MIC = 1;
    public SpeechRecognizer speechRecognizer;
    TextToSpeech tts;
    public String user_text;
    public String robot_text;
    public boolean speak_;
    public int phase = 0;
    public boolean first_end_phase=false;
    public Locale currentLocale;
    public boolean adaptation = false;
    private final String it_lang = "it-IT";
    private final String en_lang = "en-US";
    private final String de_lang = "de-DE";
    private double lastStartTime;
    private final double MIN_RESTART_INTERVAL_MS = 1000;

    public int silence_countdown = 0;

    @Override
    protected void onCreate(Bundle savedInstanceState){
        super.onCreate(savedInstanceState);
        ActivityCompat.requestPermissions(this, new String[]{Manifest.permission.RECORD_AUDIO}, 1);

        setContentView(R.layout.activity_localization);
        titleTextView = findViewById(R.id.titleTextView);

        this.checkAudioPermission();

        ImageView closeButton = findViewById(R.id.closeButton);
        closeButton.setOnClickListener(v -> onCloseClicked());
        ImageView backButton = findViewById(R.id.backButton);
        backButton.setOnClickListener(v -> onBackPressed());

        QiSDK.register(this, this);
    }

    @Override
    protected void onResume() {
        super.onResume();

        disposable = screenMachine.screenState()
                .subscribeOn(Schedulers.io())
                .observeOn(Schedulers.io())
                .subscribe(this::onScreenStateChanged);
    }

    @Override
    protected void onPause() {
        if (disposable != null && !disposable.isDisposed()){
            disposable.dispose();
        }

        super.onPause();
    }

    @Override
    protected void onDestroy(){
        QiSDK.unregister(this, this);
        if (tts != null) {
            tts.stop();
            tts.shutdown();
        }
        if (speechRecognizer != null) {
            speechRecognizer.cancel();
            speechRecognizer.destroy();
        }
        super.onDestroy();
    }

    @Override
    public void onBackPressed() {screenMachine.post(ScreenEvent.BACK);}
    public void onCloseClicked(){finishAffinity();}
    public void onBackClicked(){onBackPressed();}

    public void releaseAutonomousAbilities() {
        if (holderBack != null) {
            holderBack.release();
            holderBack = null;
        }
        if (holderBasic != null) {
            holderBasic.release();
            holderBasic = null;
        }
    }

    public void holdAutonomousAbilities() {
        if (qiContext == null) {
            Log.w(TAG, "QiContext is null in holdAutonomousAbilities()");
            return;
        }

        qiContext.getAutonomousAbilities().async()
                .holdBackgroundMovement(qiContext.getRobotContext())
                .thenConsume(futureBack -> {
                    if (futureBack.isSuccess()) {
                        Log.d(TAG, "Background movement held");
                        holderBack = futureBack.get();

                        qiContext.getAutonomousAbilities().async()
                                .holdBasicAwareness(qiContext.getRobotContext())
                                .thenConsume(futureBasic -> {
                                    if (futureBasic.isSuccess()) {
                                        Log.d(TAG, "Basic awareness held");
                                        holderBasic = futureBasic.get();
                                    } else {
                                        Log.e(TAG, "Failed to hold basic awareness", futureBasic.getError());
                                    }
                                });

                    } else {
                        Log.e(TAG, "Failed to hold background movement", futureBack.getError());
                    }
                });
    }

    @Override
    public void onRobotFocusGained(QiContext qiContext){
        this.qiContext = qiContext;
        screenMachine.post(ScreenEvent.FOCUS_GAINED);
        holdAutonomousAbilities();

        currentLocale = Locale.ITALIAN;
        this.initSpeech(qiContext);
        currentLocale = Locale.GERMAN;
        this.initSpeech(qiContext);
        currentLocale = Locale.ENGLISH;
        this.initSpeech(qiContext);
    }

    @Override
    public void onRobotFocusLost(){
        this.qiContext = null;
        releaseAutonomousAbilities();
        screenMachine.post(ScreenEvent.FOCUS_LOST);
    }

    @Override
    public void onRobotFocusRefused(String reason) {Log.e(TAG, "obRobotFocusRefused: " + reason); }

    /**
     * Show the specified fragment
     *
     * @param fragment the fragment to show
     */
    public void showFragment(@NonNull Fragment fragment){
        runOnUiThread(() ->
                getSupportFragmentManager()
                .beginTransaction()
                .replace(R.id.container, fragment)
                .addToBackStack(null) // <-- This is the missing line
                .commit());
    }

    /**
     * Provide the {@link ScreenMachine}
     *
     * @return The {@link ScreenMachine}
     */
    @NonNull
    public ScreenMachine getScreenMachine() {return screenMachine; }

    /**
     * Set the title navigation bar
     *
     * @param titleRes the string resource for the title
     */
    public void setNavigationTitle(@StringRes int titleRes){
        runOnUiThread(() -> titleTextView.setText(titleRes));
    }

    private void startLocalizationMenuString() {
        startScreen(new LocalizationMenuScreen(this, localizeManager));
    }

    private void startLocalizeScreen() {
        Log.d(TAG, "startLocalizeScreen function");
        startScreen(new LocalizeScreen(this, localizeManager));
    }

    private void startGoToGoalScreen() {
        startScreen(new GoToGoalScreen(this));
    }

    private void startGoToGoalBaselineScreen() {
        startScreen(new GoToGoalBaselineScreen(this));
    }

    private void startGoToGoalForeknowledgeScreen() {
        startScreen(new GoToGoalForeknowledgeScreen(this));
    }

    private void startGoToGoalAdaptationScreen() {
        startScreen(new GoToGoalAdaptationScreen(this));
    }

    private void startScreen(@NonNull Screen screen) {
        if (currentScreen == null) {
            doStartScreen(screen);
            return;
        }
        currentScreen.stop().andThenConsume( ignored -> doStartScreen(screen));
    }

    private void doStartScreen(@NonNull Screen screen) {
        if (qiContext != null) {
            currentScreen = screen;
            screen.start(qiContext);
        }
    }

    private void onScreenStateChanged(@NonNull ScreenState screenState) {
        Log.d(TAG, "onScreenStateChanged: " + screenState);

        switch(screenState) {
            case NONE:
                if (currentScreen != null) {
                    currentScreen.stop();
                    currentScreen = null;
                }
                break;
            case LOCALIZATION_MENU:
                startLocalizationMenuString();
                break;
            case LOCALIZE:
                startLocalizeScreen();
                break;
            case GO_TO_GOAL:
                startGoToGoalScreen();
                break;
            case GO_TO_GOAL_BASELINE:
                startGoToGoalBaselineScreen();
                break;
            case GO_TO_GOAL_FOREKNOWLEDGE:
                startGoToGoalForeknowledgeScreen();
                break;
            case GO_TO_GOAL_ADAPTATION:
                startGoToGoalAdaptationScreen();
                break;
            case END:
                runOnUiThread(this::finish);
                break;
        }
    }



    // Speech part
    private void checkAudioPermission(){
        if(ContextCompat.checkSelfPermission(this, Manifest.permission.RECORD_AUDIO) != PackageManager.PERMISSION_GRANTED){
            ActivityCompat.requestPermissions(this, new String[]{Manifest.permission.RECORD_AUDIO}, REQUEST_CODE_MIC);
        }
    }

    private void initRecognizer(QiContext qiContext){
        speechRecognizer = SpeechRecognizer.createSpeechRecognizer(LocalizationActivity.this);
        RecognitionListener listener = new RecognitionListener() {
            @Override
            public void onResults(Bundle results) {
                silence_countdown = 0;
                Log.d(TAG, "on Results function");
                ArrayList<String> matches = results.getStringArrayList(SpeechRecognizer.RESULTS_RECOGNITION);
                Log.d(TAG, "SpeechRecognizer: " + matches.toString());
                if (matches != null && !matches.isEmpty()) {
                    user_text = matches.get(0);
                }
                if (!speak_){
                    return;
                }
                if (user_text == null) {
                    user_text = "";
                }
                if (phase == 0){
                    handlePresentationPhase(qiContext);
                }
                else if(phase == 1){
                    handleChitchatPhase(qiContext);
                }
                else if (phase == 2){
                    handleNavigationPhase(qiContext);
                }
                else {
                    handleEndPhase(qiContext);
                }
            }

            @Override
            public void onReadyForSpeech(Bundle params) {
                //Log.d(TAG, "on ready for speech function");
                }

            @Override
            public void onBeginningOfSpeech() {
                //Log.d(TAG, "on beginning of speech");
            }

            @Override
            public void onRmsChanged(float rmsdB) {}

            @Override
            public void onBufferReceived(byte[] buffer) {}

            @Override
            public void onEndOfSpeech() {
                //Log.d(TAG, "on end of speech function");
                }

            @Override
            public void onError(int error) {
                Log.d(TAG, "Error number: " + error);
                switch (error) {
                    case SpeechRecognizer.ERROR_NO_MATCH:

                            silence_countdown++;
                            speechRecognizer = null;
                            initRecognizer(qiContext);
                            runOnUiThread(() -> {
                                restartListening();
                                Log.d(TAG, "Speech recognizer started");
                            });


                    case SpeechRecognizer.ERROR_NETWORK:
                        Log.e(TAG, "SpeechRecognizer: Network error (ERROR_NETWORK)");
                        runOnUiThread(() -> {
                            restartListening();
                            Log.d(TAG, "Speech recognizer started");
                        });
                        break;
                    default:
                        break;
                }
            }

            @Override
            public void onPartialResults(Bundle partialResults) {}

            @Override
            public void onEvent(int eventType, Bundle params) {}
        };
        speechRecognizer.setRecognitionListener(listener);
    }

    public void initSpeech(QiContext qiContext){

        lastStartTime = System.currentTimeMillis();
        initRecognizer(qiContext);
        // For starting listening
        //speechRecognizer.startListening(createRecognizerIntent(it_lang);
        // For speaking
        // speak("Hello! How are you?", Locale.ENGLISH);
        tts = new TextToSpeech(this, this::onInitSpeech);
        tts.setPitch(1.0f);
        tts.setSpeechRate(0.9f);
        tts.setOnUtteranceProgressListener(new UtteranceProgressListener() {
            @Override
            public void onStart(String utteranceId) {
                // Speaking starteds
                runOnUiThread(() -> {
                    speechRecognizer.stopListening();
                });
            }

            @Override
            public void onDone(String utteranceId) {
                Log.d(TAG, "OnDone after speaking called");
                // Add a delay before starting recognition
                runOnUiThread(() -> {
                        restartListening();
                        Log.d(TAG, "Speech recognizer started");
                });
                Log.d(TAG, "Speech recognizer started");

            }

            @Override
            public void onError(String utteranceId) {
                Log.e("TTS", "Error in speech");
            }
        });
        Log.d(TAG, "TTS initialized");
    }

    private void restartListening() {
        if (phase == 3 && first_end_phase)
        {
            Log.d(TAG, "End");
            return;
            }
        else if(phase == 3 && !first_end_phase)
        {
            first_end_phase = true;
        }

        try {
            runOnUiThread(
                    () -> {
                            speechRecognizer.startListening(createRecognizerIntent());
                        }
            );
        } catch (Exception e) {
            Log.e(TAG, "Stop listening failed or start listening failed", e);
        }
    }


    public void onInitSpeech(int status) {
        if (status == TextToSpeech.SUCCESS) {
            int result = tts.setLanguage(currentLocale);
            if (result == TextToSpeech.LANG_MISSING_DATA || result == TextToSpeech.LANG_NOT_SUPPORTED) {
                System.out.println("Error: Missing data or Language not Supported");
            }
            Set<Voice> voices = tts.getVoices();
            for (Voice voice : voices) {
                if (currentLocale.equals(Locale.ITALIAN) || currentLocale.equals(Locale.ENGLISH) || currentLocale.equals(Locale.GERMAN)) {
                    Log.d("TTS", "Voice: " + voice.getName() +
                            ", Locale: " + currentLocale +
                            ", Gender: " + (voice.getName().toLowerCase().contains("female")) +
                            ", Quality: " + voice.getQuality());
                }
            }
        }
    }

    public void speak(QiContext qiContext, String text, Locale language) {
        int result = tts.setLanguage(language);
        if (result == TextToSpeech.LANG_MISSING_DATA || result == TextToSpeech.LANG_NOT_SUPPORTED) {
            runOnUiThread(() -> Toast.makeText(qiContext, "Language not supported", Toast.LENGTH_SHORT).show());
            Log.d(TAG, "Language not supported");

        } else {
            Log.d(TAG, "Speak will be launched now");
            tts.speak(text, TextToSpeech.QUEUE_FLUSH, null, "utterance_" + System.currentTimeMillis());
            Log.d(TAG, "Speak has been launched");
        }
    }

    public Intent createRecognizerIntent() {
        String lang;
        if (currentLocale.equals(Locale.GERMAN)){
            lang = de_lang;
        } else if(currentLocale.equals(Locale.ITALIAN)){
            lang = it_lang;
        }
        else {
            lang = en_lang;
        }

        Intent intent = new Intent(RecognizerIntent.ACTION_RECOGNIZE_SPEECH);
        intent.putExtra(RecognizerIntent.EXTRA_LANGUAGE_MODEL, RecognizerIntent.LANGUAGE_MODEL_FREE_FORM);

        intent.putExtra(RecognizerIntent.EXTRA_LANGUAGE, lang);
        intent.putExtra(RecognizerIntent.EXTRA_PROMPT, "Speak now");

        // Optional: fine-tune silence timeout
        //intent.putExtra(RecognizerIntent.EXTRA_SPEECH_INPUT_COMPLETE_SILENCE_LENGTH_MILLIS, 2000);
        //intent.putExtra(RecognizerIntent.EXTRA_SPEECH_INPUT_MINIMUM_LENGTH_MILLIS, 1500);
        return intent;
    }

    public void fetchAndSetLanguage(QiContext qiContext, Runnable onComplete) {
        ClientManager.getLanguage(qiContext, new Function1<String, Unit>() {
            @Override
            public Unit invoke(String language) {
                if (language != null) {
                    Log.d("LanguageCallback", "Language received: " + language);
                    switch (language) {
                        case "Italian":
                            currentLocale = Locale.ITALIAN;
                            break;
                        case "German":
                            currentLocale = Locale.GERMAN;
                            break;
                        default:
                            currentLocale = Locale.ENGLISH;
                            break;
                    }
                } else {
                    Log.e("LanguageCallback", "Failed to retrieve language");
                    currentLocale = Locale.ENGLISH;
                }
                if (onComplete != null) {
                    runOnUiThread(onComplete);
                }
                return Unit.INSTANCE;
            }
        });
    }


    public Future<Void> handlePresentationPhase(QiContext qiContext) {
        if (speak_) {
            Log.d(TAG, "Presentation");

            ClientManager.postPhase(qiContext, "presentation", ok -> {
                if (ok) {
                    Log.d(TAG, "Phase changed successfully to presentation");

                    ClientManager.postRespond(qiContext, user_text, answer -> {
                        robot_text = answer;
                        if (!this.adaptation){
                            this.speak(qiContext, robot_text, currentLocale);
                        }
                        else{
                            fetchAndSetLanguage(qiContext, () -> {
                                // Now the locale is set, you can continue with TTS or recognition
                                this.speak(qiContext, robot_text, currentLocale);
                            });
                        }

                        return null;
                    });

                } else {
                    Log.d(TAG, "Phase not changed");
                }
                return null;
            });
        }
        return null;
    }

    private Future<Void> handleChitchatPhase(QiContext qiContext) {
        if (speak_) {
            Log.d(TAG, "chitchat");

            ClientManager.postPhase(qiContext, "chitchat", ok -> {
                if (ok) {
                    Log.d(TAG, "Phase changed successfully to chitchat");

                    ClientManager.postRespond(qiContext, user_text, answer -> {
                        robot_text = answer;

                        if (!this.adaptation){
                            this.speak(qiContext, robot_text, currentLocale);
                        }
                        else{
                            fetchAndSetLanguage(qiContext, () -> {
                                // Now the locale is set, you can continue with TTS or recognition
                                this.speak(qiContext, robot_text, currentLocale);
                            });
                        }
                        return null;
                    });

                } else {
                    Log.d(TAG, "Phase not changed");
                }
                return null;
            });
        }
        return null;
    }

    private Future<Void> handleNavigationPhase(QiContext qiContext) {
        if (speak_) {
            Log.d(TAG, "navigation");

            ClientManager.postPhase(qiContext, "navigation", ok -> {
                if (ok) {
                    Log.d(TAG, "Phase changed successfully to navigation");

                    ClientManager.postRespond(qiContext, user_text, answer -> {
                        robot_text = answer;

                        if (!this.adaptation){
                            this.speak(qiContext, robot_text, currentLocale);
                        }
                        else{
                            fetchAndSetLanguage(qiContext, () -> {
                                // Now the locale is set, you can continue with TTS or recognition
                                this.speak(qiContext, robot_text, currentLocale);
                            });
                        }
                        return null;
                    });

                } else {
                    Log.d(TAG, "Phase not changed");
                }
                return null;
            });
        }
        return null;
    }

    private Future<Void> handleEndPhase(QiContext qiContext) {
        if (speak_) {
            Log.d(TAG, "end");

            ClientManager.postPhase(qiContext, "end", ok -> {
                if (ok) {
                    Log.d(TAG, "Phase changed successfully to end");

                    ClientManager.postRespond(qiContext, user_text, answer -> {
                        robot_text = answer;

                        if (!this.adaptation){
                            this.speak(qiContext, robot_text, currentLocale);
                        }
                        else{
                            fetchAndSetLanguage(qiContext, () -> {
                                // Now the locale is set, you can continue with TTS or recognition
                                this.speak(qiContext, robot_text, currentLocale);
                            });
                        }
                        return null;
                    });

                } else {
                    Log.d(TAG, "Phase not changed");
                }
                return null;
            });
        }
        return null;
    }
}
