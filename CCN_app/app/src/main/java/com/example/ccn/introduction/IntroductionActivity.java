package com.example.ccn.introduction;


import android.Manifest;
import android.content.Context;
import android.content.Intent;
import android.content.SharedPreferences;
import android.content.pm.PackageManager;
import android.os.Bundle;
import android.os.Handler;
import android.os.Looper;
import android.speech.RecognitionListener;
import android.speech.SpeechRecognizer;
import android.speech.tts.TextToSpeech;
import android.speech.tts.Voice;
import android.util.Log;
import android.widget.Button;
import android.widget.ImageView;
import android.widget.Toast;

import com.aldebaran.qi.sdk.QiContext;
import com.aldebaran.qi.sdk.QiSDK;
import com.aldebaran.qi.sdk.RobotLifecycleCallbacks;
import com.aldebaran.qi.sdk.builder.SayBuilder;
import com.aldebaran.qi.sdk.design.activity.RobotActivity;
import com.aldebaran.qi.sdk.object.autonomousabilities.AutonomousAbilities;
import com.aldebaran.qi.sdk.object.autonomousabilities.AutonomousAbilityHolder;
import com.aldebaran.qi.sdk.object.context.RobotContext;
import com.aldebaran.qi.sdk.object.conversation.Say;
import com.example.ccn.R;
import com.example.ccn.menu.MenuActivity;

import androidx.annotation.NonNull;
import androidx.core.app.ActivityCompat;
import androidx.core.content.ContextCompat;

import java.util.ArrayList;
import java.util.EnumSet;
import java.util.Locale;
import java.util.Set;
import java.util.concurrent.Executors;
import java.util.concurrent.ScheduledExecutorService;
import java.util.concurrent.TimeUnit;

import butterknife.ButterKnife;
import butterknife.OnClick;

/**
 * The introduction Activity.
 */
public class IntroductionActivity extends RobotActivity implements RobotLifecycleCallbacks {

    @NonNull
    private static final String TAG = "IntroductionActivity";

    Button closeButton;
    QiContext qiContext = null;

    private ScheduledExecutorService scheduler;
    private final Handler mainHandler = new Handler(Looper.getMainLooper());
    AutonomousAbilityHolder holderBack;
    AutonomousAbilityHolder holderBasic;




    @Override
    protected void onCreate(Bundle savedInstanceState) {
        super.onCreate(savedInstanceState);

        setContentView(R.layout.activity_introduction);
        ButterKnife.bind(this);

        ImageView closeButton = findViewById(R.id.closeButton);
        closeButton.setOnClickListener(v -> onCloseClicked());

        QiSDK.register(this, this);
    }

    @Override
    protected void onDestroy() {
        QiSDK.unregister(this, this);

        super.onDestroy();
    }



    @Override
    public void onRobotFocusGained(QiContext qiContext) {
        this.qiContext = qiContext;
        Log.d(TAG, "on Robot Focus Gained");
        //holdAutonomousAbilities();
        Say say = SayBuilder.with(qiContext)
                .withResource(R.string.intro_speech)
                .build();
        Log.d(TAG, "Say built");


        SharedPreferences prefs = qiContext.getSharedPreferences("settings", Context.MODE_PRIVATE);
        prefs.edit().putString("server_url", qiContext.getString(R.string.serverUrl)).apply();

        say.async().run()
                .andThenConsume(ignored -> goToMenu());
    }

    @Override
    public void onRobotFocusLost() {
        // Not used.
        //releaseAutonomousAbilities();
    }

    @Override
    public void onRobotFocusRefused(String reason) {
        Log.e(TAG, "onRobotFocusRefused: " + reason);
    }

    public void onCloseClicked() {
        finishAffinity();
    }

    private void goToMenu() {
        runOnUiThread(() -> {
            Intent intent = new Intent(this, MenuActivity.class);
            startActivity(intent);
            finish();
        });
    }
}