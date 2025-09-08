package com.example.ccn.localization.adaptation;

import androidx.annotation.NonNull;

import io.reactivex.Observable;
import io.reactivex.subjects.BehaviorSubject;


/**
 * The state machine for {@link GoToGoalAdaptationScreen}
 */
public class GoToGoalAdaptationMachine {

    @NonNull
    private final BehaviorSubject<GoToGoalAdaptationState> subject = BehaviorSubject.createDefault(GoToGoalAdaptationState.IDLE);

    /**
     * Post an event to the machine
     *
     * @param event the event
     */
    void post(@NonNull GoToGoalAdaptationEvent event) {
        GoToGoalAdaptationState currentState = subject.getValue();
        if(currentState==null){
            throw new IllegalStateException("GoToGoalAdaptationMachine must have a GoToGoalAdaptationState to be able to handle a GoToGoalAdaptationEvent.");
        }

        GoToGoalAdaptationState newState = reduce(currentState, event);
        subject.onNext(newState);
    }

    /**
     *
     * Provide the current {@link GoToGoalAdaptationState}
     *
     * @return The current {@link GoToGoalAdaptationState}
     */
    @NonNull
    Observable<GoToGoalAdaptationState> goToGoalAdaptationState() {return subject.distinctUntilChanged(); }

    @NonNull
    private GoToGoalAdaptationState reduce(@NonNull GoToGoalAdaptationState currentState, @NonNull GoToGoalAdaptationEvent event) {
        switch (event) {
            case START:
                if (currentState.equals(GoToGoalAdaptationState.IDLE)) {
                    return GoToGoalAdaptationState.BRIEFING;
                }
                break;
            case STOP:
                return GoToGoalAdaptationState.IDLE;
            case START_GO_TO_GOAL_ADAPTATION:
                if (currentState.equals(GoToGoalAdaptationState.BRIEFING) || currentState.equals(GoToGoalAdaptationState.ERROR)) {
                    return GoToGoalAdaptationState.MOVING;
                }
                break;
            case GO_TO_GOAL_ADAPTATION_SUCCEEDED:
                if (currentState.equals(GoToGoalAdaptationState.MOVING) ) {
                    return GoToGoalAdaptationState.SUCCESS;
                }
                break;
            case GO_TO_GOAL_ADAPTATION_FAILED:
                if (currentState.equals(GoToGoalAdaptationState.MOVING) ) {
                    return GoToGoalAdaptationState.END;
                }
                break;
        }
        return currentState;
    }
}
