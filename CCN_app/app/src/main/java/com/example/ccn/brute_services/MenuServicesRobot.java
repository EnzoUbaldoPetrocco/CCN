package com.example.ccn.brute_services;

import android.os.Handler;
import android.os.Looper;
import android.util.Log;

import androidx.annotation.NonNull;
import androidx.annotation.Nullable;
import androidx.annotation.StringRes;

import com.aldebaran.qi.Future;
import com.aldebaran.qi.sdk.QiContext;
import com.aldebaran.qi.sdk.RobotLifecycleCallbacks;
import com.aldebaran.qi.sdk.builder.SayBuilder;
import com.aldebaran.qi.sdk.builder.TakePictureBuilder;
import com.aldebaran.qi.sdk.object.actuation.ExplorationMap;
import com.aldebaran.qi.sdk.object.autonomousabilities.AutonomousAbilities;
import com.aldebaran.qi.sdk.object.autonomousabilities.AutonomousAbilityHolder;
import com.aldebaran.qi.sdk.object.context.RobotContext;
import com.example.ccn.core.ClientManager;
import com.example.ccn.core.MapManager;
import com.example.ccn.utils.FutureCancellations;

import org.json.JSONObject;

import java.util.concurrent.Executors;
import java.util.concurrent.ScheduledExecutorService;
import java.util.concurrent.TimeUnit;

/**
 * The robot for {@link MenuServicesActivity}.
 */

class MenuServicesRobot implements RobotLifecycleCallbacks{

    @NonNull
    private static final String TAG = "MenuServicesRobot";


    @Nullable
    private QiContext qiContext;
    @Nullable
    private Future<Void> speech;


    private ScheduledExecutorService scheduler;
    private final Handler mainHandler = new Handler(Looper.getMainLooper());
    AutonomousAbilityHolder holderBack;
    AutonomousAbilityHolder holderBasic;

    @Override
    public void onRobotFocusGained(QiContext qiContext) {
        this.qiContext = qiContext;
        holdAutonomousAbilities();
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


    @Override
    public void onRobotFocusLost() {
        this.qiContext = null;
    }

    @Override
    public void onRobotFocusRefused(String reason) {
        Log.e(TAG, "onRobotFocusRefused: " + reason);
    }

    /**
     * MenuItem Functions
     */
    void getRobotPosition(){
        ClientManager.getRobotPosition(qiContext);
    }

    void getGoalPosition(){
        ClientManager.getGoalPosition(qiContext);
    }

    void postRobotPosition(){
        ClientManager.postRobotPosition(qiContext);
    }

    void postGoalPosition(JSONObject positionJson){
        ClientManager.postGoalPosition(qiContext, positionJson);
    }

    void postMap(){
        MapManager.retrieveMap(qiContext)
                .thenConsume(map -> ClientManager.sendMapToServer(qiContext, (ExplorationMap) map));
    }

    void getMap(){
        ClientManager.loadMapFromServer(qiContext);
    }

    void getNavigationPath(){
        ClientManager.getNavigationPath(qiContext, "false");
    }

    void getNavigationPoint(){
        ClientManager.getNavigationPoint(qiContext);
    }

    void getNavigationWithPlot(){
        ClientManager.getNavigationPathWithPlots(qiContext);
    }

    void getRobotInMap(){
        ClientManager.getRobotInMap(qiContext);
    }

    void postCameraInfo(){ ClientManager.postCameraInfo(qiContext);
    }

    void postDCameraInfo(){
        ClientManager.postDCameraInfo(qiContext);
    }

    void getRobotPositionCamera(){ ClientManager.getRobotPositionCamera(qiContext); }

    /**
     *
     * @param resId
     * @return
     */
    @NonNull
    private Future<Void> say(@StringRes int resId) {
        return FutureCancellations.cancel(speech)
                .andThenCompose(ignored -> {
                    if (qiContext == null) {
                        throw new IllegalStateException("qiContext is null");
                    }

                    Future<Void> newSpeech = SayBuilder.with(qiContext)
                            .withText((qiContext.getString(resId)))
                            .buildAsync()
                            .andThenCompose(say -> say.async().run());

                    speech = newSpeech;
                    return newSpeech;
                });
    }

    @NonNull
    private  Future<Void> cancelCurrentActions() {
        return FutureCancellations.cancel(speech);
    }

}





















