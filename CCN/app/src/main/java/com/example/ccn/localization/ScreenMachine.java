package com.example.ccn.localization;

import android.util.Log;

import androidx.annotation.NonNull;
import io.reactivex.Observable;
import io.reactivex.subjects.BehaviorSubject;

/**
 * The state machine for screens.
 */
public class ScreenMachine {

    @NonNull
    private static final String TAG = "ScreenMachine";

    @NonNull
    private final BehaviorSubject<ScreenState> subject = BehaviorSubject.createDefault(ScreenState.NONE);

    /**
     * Post an event to the machine
     *
     * @param event the event
     *
     */
    public void post(@NonNull ScreenEvent event) {
        ScreenState currentState = subject.getValue();
        Log.d(TAG, "Current state is: " + currentState.toString());
        if (currentState == null) {
            throw new IllegalStateException("ScreenMachine must have a ScreenState to be able to handle a ScreenEvent");

        }

        ScreenState newState = reduce(currentState, event);
        subject.onNext(newState);
    }

    /**
     * Provide the current {@link ScreenState}.
     *
     * @return The current {@link ScreenState}.
     */
    @NonNull
    Observable<ScreenState> screenState() {
        return subject.distinctUntilChanged();
    }

    /**Provide the current {@link ScreenState}.
     *
     * @return The current {@link ScreenState}.
     */
    @NonNull
    private  ScreenState reduce(@NonNull ScreenState currentState, @NonNull ScreenEvent event) {
        switch(event){
            case FOCUS_GAINED:
                if(currentState.equals(ScreenState.NONE)) {
                    return ScreenState.LOCALIZATION_MENU;
                }
                break;
            case FOCUS_LOST:
                return ScreenState.NONE;
            case BACK:
                switch (currentState) {
                    case LOCALIZATION_MENU:
                        return ScreenState.END;
                    case LOCALIZE:
                    case GO_TO_GOAL_ADAPTATION:
                    case GO_TO_GOAL_FOREKNOWLEDGE:
                    case GO_TO_GOAL_BASELINE:
                    case GO_TO_GOAL:
                        return ScreenState.LOCALIZATION_MENU;
                }
                break;
            case LOCALIZE_SELECTED:
                if (currentState.equals(ScreenState.LOCALIZATION_MENU)){
                    return ScreenState.LOCALIZE;
                }
                break;
            case GO_TO_GOAL_SELECTED:
                if (currentState.equals(ScreenState.LOCALIZATION_MENU)) {
                    return ScreenState.GO_TO_GOAL;
                }
                break;
            case GO_TO_GOAL_BASELINE_SELECTED:
                if (currentState.equals(ScreenState.LOCALIZATION_MENU)) {
                    return ScreenState.GO_TO_GOAL_BASELINE;
                }
                break;
            case GO_TO_GOAL_FOREKNOWLEDGE_SELECTED:
                if (currentState.equals(ScreenState.LOCALIZATION_MENU)) {
                    return ScreenState.GO_TO_GOAL_FOREKNOWLEDGE;
                }
                break;
            case GO_TO_GOAL_ADAPTATION_SELECTED:
                if (currentState.equals(ScreenState.LOCALIZATION_MENU)) {
                    return ScreenState.GO_TO_GOAL_ADAPTATION;
                }
                break;

            case LOCALIZE_END:
                if (currentState.equals(ScreenState.LOCALIZE)) {
                    return ScreenState.LOCALIZATION_MENU;
                }
                break;
            case GO_TO_GOAL_END:
                if (currentState.equals(ScreenState.GO_TO_GOAL)) {
                    return ScreenState.LOCALIZATION_MENU;
                }
                break;
            case GO_TO_GOAL_BASELINE_END:
                if (currentState.equals(ScreenState.GO_TO_GOAL_BASELINE)) {
                    return ScreenState.LOCALIZATION_MENU;
                }
                break;
            case GO_TO_GOAL_FOREKNOWLEDGE_END:
                if (currentState.equals(ScreenState.GO_TO_GOAL_FOREKNOWLEDGE)) {
                    return ScreenState.LOCALIZATION_MENU;
                }
                break;
            case GO_TO_GOAL_ADAPTATION_END:
                if (currentState.equals(ScreenState.GO_TO_GOAL_ADAPTATION)) {
                    return ScreenState.LOCALIZATION_MENU;
                }
                break;
        }
        return currentState;
    }
}
