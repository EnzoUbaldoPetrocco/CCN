package com.example.ccn.localization.adaptation;

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
public class GoToGoalAdaptationScreen implements Screen{

    @NonNull
    public final LocalizationActivity activity;

    @NonNull
    public final GoToGoalAdaptationRobot robot;

    @NonNull
    private final GoToGoalAdaptationMachine machine = new GoToGoalAdaptationMachine();

    public GoToGoalAdaptationScreen(@NonNull LocalizationActivity activity){
        this.activity = activity;
        this.robot = new GoToGoalAdaptationRobot(machine, this);
    }

    @Override
    public void start(@NonNull QiContext qiContext) {
        activity.setNavigationTitle(R.string.go_to_goal_title);

        GoToGoalAdaptationFragment fragment = GoToGoalAdaptationFragment.newInstance(this, machine);
        activity.showFragment(fragment);
        robot.start(qiContext);
    }

    @NonNull
    @Override
    public Future<Void> stop() {return robot.stop(); }

    void onGoToGoalAdaptationEnd() {activity.getScreenMachine().post(ScreenEvent.GO_TO_GOAL_ADAPTATION_END); }
}
