package com.example.ccn.localization.foreknowledge;

import androidx.annotation.NonNull;

import io.reactivex.Observable;
import io.reactivex.subjects.BehaviorSubject;


/**
 * The state machine for {@link GoToGoalForeknowledgeScreen}
 */
public class GoToGoalForeknowledgeMachine {

    @NonNull
    private final BehaviorSubject<GoToGoalForeknowledgeState> subject = BehaviorSubject.createDefault(GoToGoalForeknowledgeState.IDLE);

    /**
     * Post an event to the machine
     *
     * @param event the event
     */
    void post(@NonNull GoToGoalForeknowledgeEvent event) {
        GoToGoalForeknowledgeState currentState = subject.getValue();
        if(currentState==null){
            throw new IllegalStateException("GoToGoalForeknowledgeMachine must have a GoToGoalForeknowledgeState to be able to handle a GoToGoalForeknowledgeEvent.");
        }

        GoToGoalForeknowledgeState newState = reduce(currentState, event);
        subject.onNext(newState);
    }

    /**
     *
     * Provide the current {@link GoToGoalForeknowledgeState}
     *
     * @return The current {@link GoToGoalForeknowledgeState}
     */
    @NonNull
    Observable<GoToGoalForeknowledgeState> goToGoalForeknowledgeState() {return subject.distinctUntilChanged(); }

    @NonNull
    private GoToGoalForeknowledgeState reduce(@NonNull GoToGoalForeknowledgeState currentState, @NonNull GoToGoalForeknowledgeEvent event) {
        switch (event) {
            case START:
                if (currentState.equals(GoToGoalForeknowledgeState.IDLE)) {
                    return GoToGoalForeknowledgeState.BRIEFING;
                }
                break;
            case STOP:
                return GoToGoalForeknowledgeState.IDLE;
            case START_GO_TO_GOAL_FOREKNOWLEDGE:
                if (currentState.equals(GoToGoalForeknowledgeState.BRIEFING) || currentState.equals(GoToGoalForeknowledgeState.ERROR)) {
                    return GoToGoalForeknowledgeState.MOVING;
                }
                break;
            case GO_TO_GOAL_FOREKNOWLEDGE_SUCCEEDED:
                if (currentState.equals(GoToGoalForeknowledgeState.MOVING) ) {
                    return GoToGoalForeknowledgeState.SUCCESS;
                }
                break;
            case GO_TO_GOAL_FOREKNOWLEDGE_FAILED:
                if (currentState.equals(GoToGoalForeknowledgeState.MOVING) ) {
                    return GoToGoalForeknowledgeState.END;
                }
                break;
        }
        return currentState;
    }
}
