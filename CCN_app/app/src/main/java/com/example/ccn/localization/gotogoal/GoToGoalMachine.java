package com.example.ccn.localization.gotogoal;

import androidx.annotation.NonNull;

import io.reactivex.Observable;
import io.reactivex.subjects.BehaviorSubject;


/**
 * The state machine for {@link GoToGoalScreen}
 */
public class GoToGoalMachine {

    @NonNull
    private final BehaviorSubject<GoToGoalState> subject = BehaviorSubject.createDefault(GoToGoalState.IDLE);

    /**
     * Post an event to the machine
     *
     * @param event the event
     */
    void post(@NonNull GoToGoalEvent event) {
        GoToGoalState currentState = subject.getValue();
        if(currentState==null){
            throw new IllegalStateException("GoToOriginMachine must have a GoToOriginState to be able to handle a GoToOriginEvent.");
        }

        GoToGoalState newState = reduce(currentState, event);
        subject.onNext(newState);
    }

    /**
     *
     * Provide the current {@link GoToGoalState}
     *
     * @return The current {@link GoToGoalState}
     */
    @NonNull
    Observable<GoToGoalState> goToGoalState() {return subject.distinctUntilChanged(); }

    @NonNull
    private GoToGoalState reduce(@NonNull GoToGoalState currentState, @NonNull GoToGoalEvent event) {
        switch (event) {
            case START:
                if (currentState.equals(GoToGoalState.IDLE)) {
                    return GoToGoalState.BRIEFING;
                }
                break;
            case STOP:
                return GoToGoalState.IDLE;
            case START_GO_TO_GOAL:
                if (currentState.equals(GoToGoalState.BRIEFING) || currentState.equals(GoToGoalState.ERROR)) {
                    return GoToGoalState.MOVING;
                }
                break;
            case GO_TO_GOAL_SUCCEEDED:
                if (currentState.equals(GoToGoalState.MOVING) ) {
                    return GoToGoalState.SUCCESS;
                }
                break;
            case GO_TO_GOAL_FAILED:
                if (currentState.equals(GoToGoalState.MOVING) ) {
                    return GoToGoalState.END;
                }
                break;
        }
        return currentState;
    }
}
