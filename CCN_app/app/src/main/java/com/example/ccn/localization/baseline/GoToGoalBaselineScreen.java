package com.example.ccn.localization.baseline;

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
public class GoToGoalBaselineScreen implements Screen{

    @NonNull
    public final LocalizationActivity activity;

    @NonNull
    public final GoToGoalBaselineRobot robot;

    public GoToGoalBaselineFragment fragment;

    @NonNull
    private final GoToGoalBaselineMachine machine = new GoToGoalBaselineMachine();

    public GoToGoalBaselineScreen(@NonNull LocalizationActivity activity){
        this.activity = activity;
        this.robot = new GoToGoalBaselineRobot(machine, this);
    }

    @Override
    public void start(@NonNull QiContext qiContext) {
        activity.setNavigationTitle(R.string.go_to_goal_title);

        fragment = GoToGoalBaselineFragment.newInstance(this, machine);
        activity.showFragment(fragment);
        robot.start(qiContext);
    }

    @NonNull
    @Override
    public Future<Void> stop() {return robot.stop(); }

    void onGoToGoalBaselineEnd() {activity.getScreenMachine().post(ScreenEvent.GO_TO_GOAL_BASELINE_END); }
}
