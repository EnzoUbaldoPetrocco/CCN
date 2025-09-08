package com.example.ccn.localization.gotogoal;

import android.Manifest;
import android.app.Activity;
import android.content.Intent;
import android.content.pm.PackageManager;
import android.os.Bundle;
import android.speech.RecognitionListener;
import android.speech.RecognizerIntent;
import android.speech.tts.Voice;
import android.util.Log;
import androidx.annotation.NonNull;
import androidx.annotation.Nullable;
import androidx.annotation.StringRes;
import androidx.core.app.ActivityCompat;
import androidx.core.content.ContextCompat;

import com.aldebaran.qi.Future;
import com.aldebaran.qi.sdk.QiContext;
import com.aldebaran.qi.sdk.builder.GoToBuilder;
import com.aldebaran.qi.sdk.builder.TransformBuilder;
import com.aldebaran.qi.sdk.object.actuation.Frame;
import com.aldebaran.qi.sdk.object.geometry.Transform;
import com.example.ccn.R;
import com.example.ccn.core.ClientManager;
import com.example.ccn.localization.Robot;
import com.example.ccn.localization.Screen;
import com.example.ccn.utils.FutureCancellations;
import org.json.JSONArray;
import org.json.JSONException;
import org.json.JSONObject;

import com.aldebaran.qi.sdk.util.FutureUtils;

import java.util.ArrayList;
import java.util.Locale;
import java.util.Set;
import java.util.concurrent.CancellationException;
import java.util.concurrent.ExecutionException;
import java.util.concurrent.TimeUnit;

import io.reactivex.disposables.Disposable;
import io.reactivex.schedulers.Schedulers;

// Native Speech Recognition
import android.speech.SpeechRecognizer;

// Native TTS
import android.speech.tts.TextToSpeech;
import android.widget.Toast;

public class GoToGoalRobot implements Robot {
    @NonNull private static final String TAG = "GoToGoalRobot";
    @NonNull private final GoToGoalMachine machine;
    @NonNull private final GoToGoalScreen screen;
    @Nullable private QiContext qiContext;
    @Nullable private Disposable disposable;
    @Nullable private Future<Void> speech;
    @Nullable private Future<Void> movement;
    @Nullable private Future<Void> conversationManager;
    @Nullable private Transform map2robot;
    @Nullable private Frame mapFrame;

    private static final double MIN_MOVE_DISTANCE = 0.2;

    final long INTERVAL = 300; // ms

    final float max_speed = (float) 0.15;
    private JSONArray pathToGoal;

    int timer_time = 3;
    int index_movement = 2;
    int n_conv_phases = 3;
    String path_type = "optimal_path";

    private boolean returning = false;
    private boolean speak_ = false;


    GoToGoalRobot(@NonNull GoToGoalMachine machine, GoToGoalScreen screen) {
        this.machine = machine;
        this.screen = screen;
        this.screen.activity.speak_ = speak_;
        this.screen.activity.adaptation = false;
    }

    @NonNull
    @Override
    public Future<Void> stop() {
        machine.post(GoToGoalEvent.STOP);
        if (disposable != null && !disposable.isDisposed()) {
            disposable.dispose();
        }
        this.qiContext = null;

        return FutureCancellations.cancel(speech);
    }



    public void start(@NonNull QiContext qiContext) {
        this.qiContext = qiContext;
        Log.d(TAG, "On start");


        machine.post(GoToGoalEvent.START);
        disposable = machine.goToGoalState()
                .subscribeOn(Schedulers.io())
                .observeOn(Schedulers.io())
                .subscribe(this::onGoToGoalStateChanged);
    }


    private void goToGoalFrame() {
        if (qiContext == null) {
            Log.e(TAG, "qiContext is null");
            machine.post(GoToGoalEvent.GO_TO_GOAL_FAILED);
            return;
        }

        new Thread(() -> {
            try {
                // Optional logging
                Log.d(TAG, "Waiting 5 seconds before getting map frame...");
                Thread.sleep(7000);

            } catch (InterruptedException e) {
                Log.e(TAG, "Sleep interrupted", e);
            } catch (CancellationException e) {
                Log.e(TAG, "Error getting map frame", e);
                machine.post(GoToGoalEvent.GO_TO_GOAL_FAILED);
            }
        }).start();


        qiContext.getMapping().async().mapFrame()
                .thenConsume(mapFrameFuture -> {
                    if (!mapFrameFuture.isSuccess()) {
                        Log.e(TAG, "Failed to retrieve mapFrame");
                        machine.post(GoToGoalEvent.GO_TO_GOAL_FAILED);
                        return;
                    }
                    mapFrame = mapFrameFuture.getValue();
                    try {
                        map2robot = qiContext.getActuation().robotFrame().computeTransformWithRobotDriftCompensation(
                                mapFrame
                        ).getTransform();
                        Log.d(TAG, "First robot position: " + map2robot.toString());
                        Log.d(TAG, "First robotFrame compute Transform: " + qiContext.getActuation().robotFrame().computeTransformWithRobotDriftCompensation(
                                mapFrame
                        ));
                        ClientManager.postRobotPosition(qiContext, map2robot);
                    } catch (Exception e) {
                        Log.e(TAG, "Transform error", e);
                        machine.post(GoToGoalEvent.GO_TO_GOAL_FAILED);
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
                        machine.post(GoToGoalEvent.GO_TO_GOAL_FAILED);
                    }
                });
    }

    private void moveToNextNode(int index, JSONArray nodes) {
        qiContext.getMapping().async().mapFrame()
                .thenConsume(mapFrameFuture -> {
                    if (!mapFrameFuture.isSuccess()) {
                        Log.e(TAG, "Failed to retrieve mapFrame");
                        machine.post(GoToGoalEvent.GO_TO_GOAL_FAILED);
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
                        machine.post(GoToGoalEvent.GO_TO_GOAL_FAILED);
                        return;
                    }
                    Log.d(TAG, "Robot position w.r.t map is: " + map2robot.getTranslation().toString());
                    ClientManager.postRobotPosition(qiContext, map2robot);

                    if (index >= nodes.length()) {
                        Log.d(TAG, "Finished path. returning=" + returning);
                        if (!returning) {
                            // Just finished going to the goal

                            this.screen.activity.releaseAutonomousAbilities();
                            returning = !returning;
                            moveBackToStart();
                        } else {
                            // Just finished returning — stop here
                            this.screen.activity.holdAutonomousAbilities();
                            machine.post(GoToGoalEvent.GO_TO_GOAL_SUCCEEDED);
                        }
                        return;
                    }
                    try {
                        JSONObject current = nodes.getJSONObject(index);
                        double x = current.getDouble("x");
                        double y = current.getDouble("y");
                        assert mapFrame != null;
                        assert qiContext != null;

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
                        machine.post(GoToGoalEvent.GO_TO_GOAL_FAILED);
                    }
                    Log.d(TAG, "Done with index: " + index);
                    System.out.flush();

                });


    }


    private void moveBackToStart() {
        if (pathToGoal == null) {
            machine.post(GoToGoalEvent.GO_TO_GOAL_FAILED);
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
            if (reversedPath.length() < 2){
                map2robot = qiContext.getActuation().robotFrame().computeTransformWithRobotDriftCompensation(
                        mapFrame
                ).getTransform();
                ClientManager.postRobotPosition(qiContext, map2robot);
                rev = ClientManager.getNavigationPath(qiContext, "true");
                reversedPath = rev.getJSONArray(path_type);
            }
            Log.d(TAG, "Path reversed: " + reversedPath);
            moveToNextNode(0, reversedPath);
        } catch (JSONException e) {
            Log.e(TAG, "Error reversing path", e);
            machine.post(GoToGoalEvent.GO_TO_GOAL_FAILED);
        }

    }

    @NonNull
    private Future<Void> cancelCurrentActions() {
        return FutureCancellations.cancel(speech, movement, conversationManager);
    }

    private void onGoToGoalStateChanged(@NonNull GoToGoalState goToGoalState) {
        switch (goToGoalState) {
            case IDLE:
            case END:
                cancelCurrentActions();
                break;
            case BRIEFING:
                cancelCurrentActions();
                break;
            case MOVING:
                cancelCurrentActions()
                        .andThenConsume(ignored -> goToGoalFrame());
                break;
            case ERROR:
                cancelCurrentActions();
                Log.e(TAG, "ERROR while navigating");
                break;
            case SUCCESS:
                cancelCurrentActions()
                        .andThenConsume(ignored -> machine.post(GoToGoalEvent.SUCCESS_CONFIRMED));
                break;
        }
    }
}
