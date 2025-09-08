package com.example.ccn.localization;

import android.util.Log;

import com.aldebaran.qi.Future;
import com.aldebaran.qi.Promise;
import com.aldebaran.qi.sdk.QiContext;
import com.aldebaran.qi.sdk.builder.LocalizeBuilder;
import com.aldebaran.qi.sdk.builder.TransformBuilder;
import com.aldebaran.qi.sdk.object.actuation.Frame;
import com.aldebaran.qi.sdk.object.actuation.LocalizationStatus;
import com.aldebaran.qi.sdk.object.actuation.Localize;
import com.aldebaran.qi.sdk.object.geometry.Transform;
import com.example.ccn.core.MapManager;
import com.example.ccn.localization.localize.LocalizeMachine;
import com.example.ccn.utils.FutureCancellations;

import java.util.Objects;
import java.util.concurrent.atomic.AtomicBoolean;

import androidx.annotation.NonNull;
import androidx.annotation.Nullable;
import androidx.annotation.StringRes;

import com.example.ccn.core.ClientManager;

import org.json.JSONException;
import org.json.JSONObject;

/**
 * Manager that starts the localization.
 */

public class LocalizeManager {
    @NonNull
    private static final String TAG = "LocalizeManager";
    @NonNull
    private final AtomicBoolean isLocalized = new AtomicBoolean(false);
    @Nullable
    private Localize localize;
    @Nullable
    private Future<Void> localization;

    /**
     * Indicate if the robot is localized or not
     *
     * @return {@code true} if the robot is localized, {@code false} otherwise
     */
    public boolean isLocalized() {
        return isLocalized.get();
    }

    /**
     * Indicates if the map is loaded.
     *
     * @return {@code true} if the map is loaded, {@code false} otherwise
     */
    public boolean mapIsLoaded() {
        return localize != null;
    }

    /**
     * Load the map and create the {@link Localize} action
     *
     * @param qiContext the qi Context
     * @return A {@link Future} wrapping the operation.
     */
    @NonNull
    public Future<Void> loadMap(@NonNull QiContext qiContext){
        return MapManager.retrieveMap(qiContext)
                .andThenCompose(map -> {
                    Log.d(TAG, "Map retrieved successfully");
                    //ClientManager.sendMapToServer(qiContext, map);
                    return LocalizeBuilder.with(qiContext)
                            .withMap(map)
                            .buildAsync();
                })
                .andThenConsume(loc -> {
                    Log.d(TAG, "Localize built successfully");
                    localize = loc;
                });
    }

    /**
     * Localize the robot. This method starts the localization and waits for the robot to be localized
     * Once localized, the operation is coionsidered as successful and the robot stays localized until the localization is cancelled or encounters and error
     *
     * @return A {@link Future} wrapping the operation
     * If the {@link Localize} action is cancelled before that, the operation is cancelled.
     * If the {@link Localize} action encounters and error before that, the operation fails
     */
    @NonNull
    public Future<Void> localizeRobot(@NonNull QiContext qiContext) {
        Promise<Void> promise = new Promise<>();
        FutureCancellations.cancel(localization)
                .andThenCompose(ignored -> {
                    if (localize == null) {
                        throw new IllegalStateException("localize is null");
                    }

                    localize.addOnStatusChangedListener(status -> {
                        if (status == LocalizationStatus.LOCALIZED) {
                            Log.d(TAG, "Robot is localized");
                            isLocalized.set(true);
                            if(!promise.getFuture().isDone()){
                                ClientManager.postRobotPosition(qiContext);
                                //Once the robot is localized, consider the operation as a success
                                promise.setValue(null);
                            }
                        }
                    });

                    Log.d(TAG, "Running Localize...");
                    localization = localize.async().run();
                    
                    return localization;
                })
                .thenConsume(future -> {
                    isLocalized.set(false);

                    if(localize!=null){
                        localize.removeAllOnStatusChangedListeners();
                    }
                    if (future.hasError()){
                        Log.e(TAG, "Error while localizing", future.getError());
                        if(!promise.getFuture().isDone()){
                            //Consider the operation as a failure.
                            promise.setError(future.getErrorMessage());
                        }
                    } else if (future.isCancelled()) {
                        if(!promise.getFuture().isDone()) {
                            //Consider the operation has been cancelled.
                            promise.setCancelled();;
                        }
                    }

                });

        // try ClientManager Localization and compare them
        JSONObject robotPositionJson = ClientManager.getRobotPositionCamera(qiContext);
        try {
            Log.d(TAG, "X: " + robotPositionJson.getDouble("x") + "y: "  + robotPositionJson.getDouble("x") + "theta: " + + robotPositionJson.getDouble("theta") );
        } catch (JSONException e) {
            e.printStackTrace();
        }

        //Return the future associated with the promise
        return promise.getFuture();
    }


    /**
     * Localize the robot with an hint. This method starts the localization and waits for the robot to be localized
     * Once localized, the operation is coionsidered as successful and the robot stays localized until the localization is cancelled or encounters and error
     *
     * @return A {@link Future} wrapping the operation
     * If the {@link Localize} action is cancelled before that, the operation is cancelled.
     * If the {@link Localize} action encounters and error before that, the operation fails
     */
    @NonNull
    public Future<Void> localizeRobot(@NonNull QiContext qiContext, JSONObject hint) throws JSONException {
        Promise<Void> promise = new Promise<>();
        Transform transformHint = TransformBuilder.create().from2DTransform(hint.getDouble("x"), hint.getDouble("y"), hint.getDouble("theta"));

        FutureCancellations.cancel(localization)
                .andThenCompose(ignored -> {
                    if (localize == null) {
                        throw new IllegalStateException("localize is null");
                    }

                    localize.addOnStatusChangedListener(status -> {
                        if (status == LocalizationStatus.LOCALIZED) {
                            Log.d(TAG, "Robot is localized");
                            isLocalized.set(true);
                            if(!promise.getFuture().isDone()){
                                ClientManager.postRobotPosition(qiContext);
                                //Once the robot is localized, consider the operation as a success
                                promise.setValue(null);
                            }
                        }
                    });

                    Log.d(TAG, "Running Localize...");
                    if (hint.toString()!="{}"){
                        localization = localize.async().runWithLocalizationHint(transformHint);
                    }
                    else{
                        localization = localize.async().run();
                    }
                    return localization;
                })
                .thenConsume(future -> {
                    isLocalized.set(false);

                    if(localize!=null){
                        localize.removeAllOnStatusChangedListeners();
                    }
                    if (future.hasError()){
                        Log.e(TAG, "Error while localizing", future.getError());
                        if(!promise.getFuture().isDone()){
                            //Consider the operation as a failure.
                            promise.setError(future.getErrorMessage());
                        }
                    } else if (future.isCancelled()) {
                        if(!promise.getFuture().isDone()) {
                            //Consider the operation has been cancelled.
                            promise.setCancelled();;
                        }
                    }
                });

        // try ClientManager Localization and compare them
        JSONObject robotPositionJson = ClientManager.getRobotPositionCamera(qiContext);
        try {
            Log.d(TAG, "X: " + robotPositionJson.getDouble("x") + "y: "  + robotPositionJson.getDouble("x") + "theta: " + + robotPositionJson.getDouble("theta") );
        } catch (JSONException e) {
            e.printStackTrace();
        }


        //Return the future associated with the promise
        return promise.getFuture();
    }

}
