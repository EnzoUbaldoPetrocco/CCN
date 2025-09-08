package com.example.ccn.localization.gotogoal;

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
public class GoToGoalScreen implements Screen{

    @NonNull
    public final LocalizationActivity activity;

    @NonNull
    public final GoToGoalRobot robot;

    @NonNull
    private final GoToGoalMachine machine = new GoToGoalMachine();

    public GoToGoalScreen(@NonNull LocalizationActivity activity){
        this.activity = activity;
        this.robot = new GoToGoalRobot(machine, this);
    }

    @Override
    public void start(@NonNull QiContext qiContext) {
        activity.setNavigationTitle(R.string.go_to_goal_title);

        GoToGoalFragment fragment = GoToGoalFragment.newInstance(this, machine);
        activity.showFragment(fragment);
        robot.start(qiContext);
    }

    @NonNull
    @Override
    public Future<Void> stop() {return robot.stop(); }

    void onGoToGoalEnd() {activity.getScreenMachine().post(ScreenEvent.GO_TO_GOAL_END); }
}
