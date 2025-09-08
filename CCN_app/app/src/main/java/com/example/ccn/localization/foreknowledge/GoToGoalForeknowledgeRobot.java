package com.example.ccn.localization.foreknowledge;

import android.os.Bundle;
import android.speech.SpeechRecognizer;
import android.util.Log;

import androidx.annotation.NonNull;
import androidx.annotation.Nullable;

import com.aldebaran.qi.Future;
import com.aldebaran.qi.sdk.QiContext;
import com.aldebaran.qi.sdk.builder.GoToBuilder;
import com.aldebaran.qi.sdk.builder.TransformBuilder;
import com.aldebaran.qi.sdk.object.actuation.Frame;
import com.aldebaran.qi.sdk.object.geometry.Transform;
import com.aldebaran.qi.sdk.util.FutureUtils;
import com.example.ccn.R;
import com.example.ccn.core.ClientManager;
import com.example.ccn.localization.Robot;
import com.example.ccn.localization.foreknowledge.GoToGoalForeknowledgeEvent;
import com.example.ccn.utils.FutureCancellations;

import org.json.JSONArray;
import org.json.JSONException;
import org.json.JSONObject;

import java.util.ArrayList;
import java.util.Locale;
import java.util.concurrent.CancellationException;
import java.util.concurrent.TimeUnit;

import io.reactivex.disposables.Disposable;
import io.reactivex.schedulers.Schedulers;

// Native Speech Recognition

// Native TTS


public class GoToGoalForeknowledgeRobot implements Robot {
    @NonNull private static final String TAG = "GoToGoalForeknowledgeRobot";
    @NonNull private final GoToGoalForeknowledgeMachine machine;
    @NonNull private final GoToGoalForeknowledgeScreen screen;
    @Nullable private QiContext qiContext;
    @Nullable private Disposable disposable;
    @Nullable private Future<Void> speech;
    @Nullable private Future<Void> movement;
    @Nullable private Future<Void> conversationManager;
    @Nullable private Transform map2robot;
    @Nullable private Frame mapFrame;

    private static final double MIN_MOVE_DISTANCE = 0.2;

    final long INTERVAL = 300; // ms

    final float max_speed = (float) 0.07;
    private JSONArray pathToGoal;

    int timer_time = 3;
    int index_movement = 2;
    int n_conv_phases = 3;
    private int phase = 1;
    String path_type = "optimal_path";

    private boolean returning = false;
    private boolean speak_ = true;

    public Locale currentLocale;
    public String culture;
    private final String it_lang = "it-IT";
    private final String en_lang = "en-US";
    private final String de_lang = "de-DE";

    public String user_text;
    public String robot_text;

    private long initInterval = 12000;


    GoToGoalForeknowledgeRobot(@NonNull GoToGoalForeknowledgeMachine machine, GoToGoalForeknowledgeScreen screen) {
        this.machine = machine;
        this.screen = screen;
    }

    @NonNull
    @Override
    public Future<Void> stop() {
        machine.post(GoToGoalForeknowledgeEvent.STOP);
        if (disposable != null && !disposable.isDisposed()) {
            disposable.dispose();
        }
        this.qiContext = null;

        return FutureCancellations.cancel(speech);
    }



    public void start(@NonNull QiContext qiContext) {
        this.qiContext = qiContext;
        Log.d(TAG, "On start");

        this.screen.activity.speak_=speak_;
        this.screen.activity.phase = phase;
        this.screen.activity.adaptation = false;
        this.screen.activity.first_end_phase = false;

        machine.post(GoToGoalForeknowledgeEvent.START);
        disposable = machine.goToGoalForeknowledgeState()
                .subscribeOn(Schedulers.io())
                .observeOn(Schedulers.io())
                .subscribe(this::onGoToGoalForeknowledgeStateChanged);
    }


    private void goToGoalForeknowledgeFrame() {
        if (qiContext == null) {
            Log.e(TAG, "qiContext is null");
            machine.post(GoToGoalForeknowledgeEvent.GO_TO_GOAL_FOREKNOWLEDGE_FAILED);
            return;
        }

        this.screen.activity.currentLocale = currentLocale;
        this.screen.activity.adaptation = false;
        this.screen.activity.runOnUiThread(() -> screen.activity.initSpeech(qiContext));
        this.screen.activity.silence_countdown = 0;

        if (currentLocale == Locale.GERMAN){
            this.screen.activity.speak(qiContext, qiContext.getString(R.string.presentation_de), currentLocale);
        } else if(currentLocale == Locale.ITALIAN){
            this.screen.activity.speak(qiContext, qiContext.getString(R.string.presentation_it), currentLocale);
        }
        else {
            this.screen.activity.speak(qiContext, qiContext.getString(R.string.presentation_en), currentLocale);
        }


        new Thread(() -> {
            try {
                // Optional logging
                Log.d(TAG, "Waiting 5 seconds before getting map frame...");
                Thread.sleep(initInterval);

                ClientManager.reset(qiContext, success -> {
                    if (success) {
                        Log.d(TAG, "Conversation Manager reset successfully!");

                    } else {
                        Log.e(TAG, "Failed to reset Conversation Manager.");
                    }
                    ClientManager.postParadigm(qiContext, culture, "foreknowledge", successParadigm -> {
                        if (successParadigm) {
                            Log.d(TAG, "Phase posted successfully!");
                        } else {
                            Log.e(TAG, "Failed to post phase.");
                        }
                        qiContext.getMapping().async().mapFrame()
                                .thenConsume(mapFrameFuture -> {
                                    if (!mapFrameFuture.isSuccess()) {
                                        Log.e(TAG, "Failed to retrieve mapFrame");
                                        machine.post(GoToGoalForeknowledgeEvent.GO_TO_GOAL_FOREKNOWLEDGE_FAILED);
                                        return;
                                    }
                                    mapFrame = mapFrameFuture.getValue();
                                    try {
                                        map2robot = qiContext.getActuation().robotFrame().computeTransformWithRobotDriftCompensation(
                                                mapFrame
                                        ).getTransform();
                                        ClientManager.postRobotPosition(qiContext, map2robot);
                                    } catch (Exception e) {
                                        Log.e(TAG, "Transform error", e);
                                        machine.post(GoToGoalForeknowledgeEvent.GO_TO_GOAL_FOREKNOWLEDGE_FAILED);
                                        return;
                                    }
                                    JSONObject navigationPathObject = ClientManager.getNavigationPath(qiContext, "false");

                                    try {
                                        pathToGoal = navigationPathObject.getJSONArray(path_type);
                                        Log.d(TAG, "Path to goal is: " + pathToGoal);
                                        if (pathToGoal.length() < 2) {
                                            map2robot = qiContext.getActuation().robotFrame().computeTransformWithRobotDriftCompensation(
                                                    mapFrame
                                            ).getTransform();
                                            ClientManager.postRobotPosition(qiContext, map2robot);
                                            navigationPathObject = ClientManager.getNavigationPath(qiContext, "false");
                                            pathToGoal = navigationPathObject.getJSONArray(path_type);
                                            Log.d(TAG, "Path to goal is: " + pathToGoal);
                                        }
                                        // Start navigation
                                        this.screen.activity.releaseAutonomousAbilities();
                                        moveToNextNode(1, pathToGoal);

                                    } catch (JSONException e) {
                                        Log.e(TAG, "Failed to parse path JSON", e);
                                        machine.post(GoToGoalForeknowledgeEvent.GO_TO_GOAL_FOREKNOWLEDGE_FAILED);
                                    }
                                });
                        return null;
                    });
                    return null;
                });

            } catch (InterruptedException e) {
                Log.e(TAG, "Sleep interrupted", e);
            } catch (CancellationException e) {
                Log.e(TAG, "Error getting map frame", e);
                machine.post(GoToGoalForeknowledgeEvent.GO_TO_GOAL_FOREKNOWLEDGE_FAILED);
            }
        }).start();



    }

    private void moveToNextNode(int index, JSONArray nodes) {
        map2robot = qiContext.getActuation().robotFrame().computeTransformWithRobotDriftCompensation(
                mapFrame
        ).getTransform();
        Log.d(TAG, "Robot position w.r.t map is: " + map2robot.getTranslation().toString());
        ClientManager.postRobotPosition(qiContext, map2robot);
        if (index >= nodes.length()) {
            Log.d(TAG, "Finished path. returning=" + returning);
            if (!returning) {
                // Just finished going to the goal
                if (speak_){
                    Log.d(TAG, "Just finished going to the goal");
                    phase = 2;
                    this.screen.activity.phase = phase;
                }
                this.screen.activity.releaseAutonomousAbilities();
                returning = !returning;
                moveBackToStart();
            } else {
                // Just finished returning — stop here
                if (speak_){
                    Log.d(TAG, "Just finished returning — stop here");
                    phase = 3;
                    this.screen.activity.phase = phase;
                }
                this.screen.activity.holdAutonomousAbilities();
                machine.post(GoToGoalForeknowledgeEvent.GO_TO_GOAL_FOREKNOWLEDGE_SUCCEEDED);
            }
            return;
        }
        try {
            JSONObject current = nodes.getJSONObject(index);
            double x = current.getDouble("x");
            double y = current.getDouble("y");
            assert mapFrame != null;
            assert qiContext != null;
            map2robot = qiContext.getActuation().robotFrame().computeTransformWithRobotDriftCompensation(
                    mapFrame
            ).getTransform();
            Log.d(TAG, "Robot position w.r.t map is: " + map2robot.getTranslation().toString());
            ClientManager.postRobotPosition(qiContext, map2robot);
            double robot_x = map2robot.getTranslation().getX();
            double robot_y = map2robot.getTranslation().getY();
            double robot_theta = Math.atan2(robot_y - y, robot_x - x);

            double dx = robot_x - x;
            double dy = robot_y - y;
            double distance = Math.sqrt(dx * dx + dy * dy);
            if (distance < MIN_MOVE_DISTANCE) {
                Log.d(TAG, "Skipping node: " + index + ": x=" + x + ", y=" + y);
                moveToNextNode(index + 1, nodes);
                return;
            }

            Transform transform = TransformBuilder.create().from2DTransform(x, y, robot_theta);
            Frame goalFrame = mapFrame.makeAttachedFrame(transform).frame();
            Log.d(TAG, "Moving to point " + index + ": x=" + x + ", y=" + y + ", theta=" + robot_theta);



            GoToBuilder.with(qiContext)
                    .withFrame(goalFrame)
                    //.withFinalOrientationPolicy(OrientationPolicy.FREE_ORIENTATION)
                    .withMaxSpeed(max_speed)
                    .buildAsync()
                    .andThenCompose(goTo -> {
                        movement = goTo.async().run();
                        return movement;
                    })
                    .thenConsume(future -> {
                        float oneThird = ((float )nodes.length()) / ((float)n_conv_phases);
                        if (phase == 0 || phase == 1) {
                            phase++;
                        }
                        this.screen.activity.phase = phase;
                        if(!returning && index < 2* oneThird){
                            this.screen.activity.holdAutonomousAbilities();
                        }
                        Log.d(TAG, "Reached point " + index + ": x=" + x + ", y=" + y + ", theta=" + robot_theta);
                        if (index + index_movement >= nodes.length()){
                            if (index <= nodes.length() - 2){
                                moveToNextNode(index + 1, nodes);
                            }
                            else{
                                moveToNextNode(nodes.length(), nodes);
                            }
                        }
                        else{
                            moveToNextNode(index + index_movement, nodes);
                        }

                    });

        } catch (Exception e) {
            Log.e(TAG, "Failed to reach point", e);
            machine.post(GoToGoalForeknowledgeEvent.GO_TO_GOAL_FOREKNOWLEDGE_FAILED);
        }
        Log.d(TAG, "Done with index: " + index);
        System.out.flush();
    }


    private void moveBackToStart() {
        if (pathToGoal == null) {
            machine.post(GoToGoalForeknowledgeEvent.GO_TO_GOAL_FOREKNOWLEDGE_FAILED);
            return;
        }
        try {
            map2robot = qiContext.getActuation().robotFrame().computeTransformWithRobotDriftCompensation(
                    mapFrame
            ).getTransform();
            Log.d(TAG, "Robot position w.r.t map is: " + map2robot.getTranslation().toString());
            ClientManager.postRobotPosition(qiContext, map2robot);
            JSONObject rev = ClientManager.getNavigationPath(qiContext, "true");
            JSONArray reversedPath = rev.getJSONArray(path_type);
            Log.d(TAG, "Path reversed: " + reversedPath);
            if (reversedPath.length() < 2){
                map2robot = qiContext.getActuation().robotFrame().computeTransformWithRobotDriftCompensation(
                        mapFrame
                ).getTransform();
                ClientManager.postRobotPosition(qiContext, map2robot);
                rev = ClientManager.getNavigationPath(qiContext, "true");
                reversedPath = rev.getJSONArray(path_type);
            }
            moveToNextNode(0, reversedPath);
        } catch (JSONException e) {
            Log.e(TAG, "Error reversing path", e);
            machine.post(GoToGoalForeknowledgeEvent.GO_TO_GOAL_FOREKNOWLEDGE_FAILED);
        }

    }




    @NonNull
    private Future<Void> cancelConversation(){
        return FutureCancellations.cancel(conversationManager);
    }

    @NonNull
    private Future<Void> cancelCurrentActions() {
        return FutureCancellations.cancel(speech, movement, conversationManager);
    }

    private void onGoToGoalForeknowledgeStateChanged(@NonNull GoToGoalForeknowledgeState goToGoalForeknowledgeState) {
        switch (goToGoalForeknowledgeState) {
            case IDLE:
            case END:
                cancelCurrentActions();
                break;
            case BRIEFING:
                cancelCurrentActions();
                break;
            case MOVING:
                cancelCurrentActions()
                        .andThenConsume(ignored -> goToGoalForeknowledgeFrame());
                break;
            case ERROR:
                cancelCurrentActions();
                Log.e(TAG, "ERROR while navigating");
                break;
            case SUCCESS:
                cancelCurrentActions()
                        .andThenConsume(ignored -> machine.post(GoToGoalForeknowledgeEvent.SUCCESS_CONFIRMED));
                break;
        }
    }
}
