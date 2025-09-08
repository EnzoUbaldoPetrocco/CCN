package com.example.ccn.localization.foreknowledge;

import androidx.annotation.NonNull;

import com.aldebaran.qi.Future;
import com.aldebaran.qi.sdk.QiContext;
import com.example.ccn.R;
import com.example.ccn.localization.LocalizationActivity;
import com.example.ccn.localization.Screen;
import com.example.ccn.localization.ScreenEvent;


/**
 * The go to origin screen.
 */
public class GoToGoalForeknowledgeScreen implements Screen{

    @NonNull
    public final LocalizationActivity activity;

    @NonNull
    public final GoToGoalForeknowledgeRobot robot;

    @NonNull
    private final GoToGoalForeknowledgeMachine machine = new GoToGoalForeknowledgeMachine();

    public GoToGoalForeknowledgeScreen(@NonNull LocalizationActivity activity){
        this.activity = activity;
        this.robot = new GoToGoalForeknowledgeRobot(machine, this);
    }

    @Override
    public void start(@NonNull QiContext qiContext) {
        activity.setNavigationTitle(R.string.go_to_goal_title);

        GoToGoalForeknowledgeFragment fragment = GoToGoalForeknowledgeFragment.newInstance(this, machine);
        activity.showFragment(fragment);
        robot.start(qiContext);
    }

    @NonNull
    @Override
    public Future<Void> stop() {return robot.stop(); }

    void onGoToGoalForeknowledgeEnd() {activity.getScreenMachine().post(ScreenEvent.GO_TO_GOAL_FOREKNOWLEDGE_END); }
}
