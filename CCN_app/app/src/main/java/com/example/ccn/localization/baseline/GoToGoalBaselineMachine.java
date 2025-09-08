package com.example.ccn.localization.baseline;

import androidx.annotation.NonNull;

import io.reactivex.Observable;
import io.reactivex.subjects.BehaviorSubject;


/**
 * The state machine for {@link GoToGoalBaselineScreen}
 */
public class GoToGoalBaselineMachine {

    @NonNull
    private final BehaviorSubject<GoToGoalBaselineState> subject = BehaviorSubject.createDefault(GoToGoalBaselineState.IDLE);

    /**
     * Post an event to the machine
     *
     * @param event the event
     */
    void post(@NonNull GoToGoalBaselineEvent event) {
        GoToGoalBaselineState currentState = subject.getValue();
        if(currentState==null){
            throw new IllegalStateException("GoToGoalBaselineMachine must have a GoToGoalBaselineState to be able to handle a GoToGoalBaselineEvent.");
        }

        GoToGoalBaselineState newState = reduce(currentState, event);
        subject.onNext(newState);
    }

    /**
     *
     * Provide the current {@link GoToGoalBaselineState}
     *
     * @return The current {@link GoToGoalBaselineState}
     */
    @NonNull
    Observable<GoToGoalBaselineState> goToGoalBaselineState() {return subject.distinctUntilChanged(); }

    @NonNull
    private GoToGoalBaselineState reduce(@NonNull GoToGoalBaselineState currentState, @NonNull GoToGoalBaselineEvent event) {
        switch (event) {
            case START:
                if (currentState.equals(GoToGoalBaselineState.IDLE)) {
                    return GoToGoalBaselineState.BRIEFING;
                }
                break;
            case STOP:
                return GoToGoalBaselineState.IDLE;
            case START_GO_TO_GOAL_BASELINE:
                if (currentState.equals(GoToGoalBaselineState.BRIEFING) || currentState.equals(GoToGoalBaselineState.ERROR)) {
                    return GoToGoalBaselineState.MOVING;
                }
                break;
            case GO_TO_GOAL_BASELINE_SUCCEEDED:
                if (currentState.equals(GoToGoalBaselineState.MOVING) ) {
                    return GoToGoalBaselineState.SUCCESS;
                }
                break;
            case GO_TO_GOAL_BASELINE_FAILED:
                if (currentState.equals(GoToGoalBaselineState.MOVING) ) {
                    return GoToGoalBaselineState.END;
                }
                break;
        }
        return currentState;
    }
}
