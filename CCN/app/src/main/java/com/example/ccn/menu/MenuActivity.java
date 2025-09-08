package com.example.ccn.menu;


import android.content.Intent;
import android.content.SharedPreferences;
import android.os.Bundle;
import android.os.Handler;
import android.os.Looper;
import android.util.Log;
import android.widget.Button;
import android.widget.EditText;
import android.widget.ImageView;
import android.widget.RadioButton;
import android.widget.Toast;

import com.aldebaran.qi.Future;
import com.aldebaran.qi.sdk.QiContext;
import com.aldebaran.qi.sdk.QiSDK;
import com.aldebaran.qi.sdk.RobotLifecycleCallbacks;
import com.aldebaran.qi.sdk.builder.HolderBuilder;
import com.aldebaran.qi.sdk.design.activity.RobotActivity;
import com.aldebaran.qi.sdk.object.autonomousabilities.AutonomousAbilities;
import com.aldebaran.qi.sdk.object.autonomousabilities.AutonomousAbilityHolder;
import com.aldebaran.qi.sdk.object.context.RobotContext;
import com.aldebaran.qi.sdk.object.holder.AutonomousAbilitiesType;
import com.aldebaran.qi.sdk.object.holder.Holder;
import com.example.ccn.R;
import com.example.ccn.brute_services.MenuServicesActivity;
import com.example.ccn.localization.LocalizationActivity;
import com.example.ccn.mapping.MappingActivity;

import java.util.concurrent.Executors;
import java.util.concurrent.ScheduledExecutorService;
import java.util.concurrent.TimeUnit;

import androidx.annotation.NonNull;
import androidx.annotation.Nullable;
import butterknife.ButterKnife;
import io.reactivex.Single;
import io.reactivex.disposables.Disposable;
import io.reactivex.schedulers.Schedulers;
import com.example.ccn.core.ClientManager;

/**
 * The menu Activity.
 */
public class MenuActivity extends RobotActivity implements RobotLifecycleCallbacks {

    @NonNull
    private static final String TAG = "MenuActivity";

    @NonNull
    private static final String START_BOOKMARK_NAME = "start";
    @NonNull
    private static final String CREATE_BOOKMARK_NAME = "create";
    @NonNull
    private static final String CREATE_END_BOOKMARK_NAME = "create_end";
    @NonNull
    private static final String USE_BOOKMARK_NAME = "use";
    @NonNull
    private static final String USE_END_BOOKMARK_NAME = "use_end";
    @NonNull
    private static final String MAP_BOOKMARK_NAME = "map";
    @NonNull
    private static final String START_TIMER_BOOKMARK_NAME = "start_timer";
    @NonNull
    private static final String STOP_TIMER_BOOKMARK_NAME = "stop_timer";


    RadioButton createMapButton;
    RadioButton useMapButton;
    Button saveUrlButton;
    EditText serverUrlEditText;

    @Nullable
    QiContext qiContext;
    @Nullable
    Disposable timerDisposable;

    private Holder holder;

    private ScheduledExecutorService scheduler;
    private final Handler mainHandler = new Handler(Looper.getMainLooper());
    AutonomousAbilityHolder holderBack;
    AutonomousAbilityHolder holderBasic;

    @Override
    protected void onCreate(Bundle savedInstanceState) {
        super.onCreate(savedInstanceState);

        setContentView(R.layout.activity_menu);
        ButterKnife.bind(this);
        createMapButton = findViewById(R.id.createMapButton);
        useMapButton = findViewById(R.id.useMapButton);
        saveUrlButton = findViewById(R.id.saveUrlButton);
        ImageView closeButton = findViewById(R.id.closeButton);

        createMapButton.setOnClickListener(v -> {
            disableButtons();
            startMappingActivity();

        });

        useMapButton.setOnClickListener(v -> {
            disableButtons();
            startLocalizationActivity();
        });


        serverUrlEditText = findViewById(R.id.serverUrlEditText);
        saveUrlButton = findViewById(R.id.saveUrlButton);

        saveUrlButton.setOnClickListener(v -> {
            String newUrl = serverUrlEditText.getText().toString().trim();
            if (!newUrl.isEmpty()) {
                SharedPreferences prefs = getSharedPreferences("settings", MODE_PRIVATE);
                prefs.edit().putString("server_url", newUrl).apply();
                Toast.makeText(this, "Server URL saved!", Toast.LENGTH_SHORT).show();
            } else {
                Toast.makeText(this, "Please enter a valid URL.", Toast.LENGTH_SHORT).show();
            }
        });

        closeButton.setOnClickListener(v -> finishAffinity());
        QiSDK.register(this, this);
    }

    @Override
    protected void onResume() {
        super.onResume();

        createMapButton.setChecked(false);
        useMapButton.setChecked(false);

        createMapButton.setEnabled(true);
        useMapButton.setEnabled(true);
    }

    @Override
    protected void onDestroy() {
        QiSDK.unregister(this, this);
        super.onDestroy();
    }

    private void holdAutonomousAbilities() {
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

    private void releaseAutonomousAbilities() {
        if (holderBack != null) {
            holderBack.release();
            holderBack = null;
        }
        if (holderBasic != null) {
            holderBasic.release();
            holderBasic = null;
        }
    }


    @Override
    public void onRobotFocusGained(QiContext qiContext) {
        this.qiContext = qiContext;
        holdAutonomousAbilities();
        //sendPicture2Server(qiContext);

        startTimer();
    }

    @Override
    public void onRobotFocusLost() {
        stopTimer();
        releaseAutonomousAbilities();
    }

    @Override
    public void onRobotFocusRefused(String reason) {
        Log.e(TAG, "onRobotFocusRefused: " + reason);
    }


    private void disableButtons() {
        runOnUiThread(() -> {
            createMapButton.setEnabled(false);
            useMapButton.setEnabled(false);
        });
    }

    private void startLocalizationActivity() {
        startActivity(new Intent(this, LocalizationActivity.class));
    }

    private void startMappingActivity() {
        startActivity(new Intent(this, MappingActivity.class));
    }

    private void startMenuServicesActivity() {
        startActivity(new Intent(this, MenuServicesActivity.class));
    }

    private void sendPicture2Server( QiContext qiContext) {
        ClientManager.postCameraInfo(qiContext);
    }

    private void getRobotEstimatedPositionFromServer( QiContext qiContext) {
        ClientManager.getRobotPositionCamera(qiContext);
    }

    private void startTimer() {
        timerDisposable = Single.timer(5, TimeUnit.SECONDS)
                .observeOn(Schedulers.io())
                .subscribeOn(Schedulers.io())
                .subscribe(ignored -> {
                    stopTimer();
                });
    }

    private void stopTimer() {
        if (timerDisposable != null && !timerDisposable.isDisposed()) {
            timerDisposable.dispose();
        }
    }
}