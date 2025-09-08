package com.example.ccn.localization.localizationmenu;

import android.util.Log;

import com.aldebaran.qi.Future;
import com.aldebaran.qi.sdk.QiContext;
import com.example.ccn.R;
import com.example.ccn.localization.LocalizationActivity;
import com.example.ccn.localization.LocalizeManager;
import com.example.ccn.localization.Screen;
import com.example.ccn.localization.ScreenEvent;

import androidx.annotation.NonNull;
import androidx.annotation.Nullable;


/**
 * The localization menu screen
 */
public class LocalizationMenuScreen implements Screen {

    @NonNull
    private static final String TAG = "LocalizationMenuScreen";

    @NonNull
    private final LocalizationActivity activity;
    @NonNull
    private final LocalizationMenuRobot robot;
    @Nullable
    private LocalizationMenuFragment localizationMenuFragment;

    public LocalizationMenuScreen(@NonNull LocalizationActivity activity, @NonNull LocalizeManager localizeManager) {

        this.activity = activity;
        this.robot = new LocalizationMenuRobot(this, localizeManager);
    }

    @Override
    public void start(@NonNull QiContext qiContext) {
        activity.setNavigationTitle(R.string.localization_menu_title);

        localizationMenuFragment = LocalizationMenuFragment.newInstance(this);
        activity.showFragment(localizationMenuFragment);
    }

    @NonNull
    @Override
    public Future<Void> stop() {return robot.stop(); }


    void disableChoices() {
        if (localizationMenuFragment != null) {
            localizationMenuFragment.disableChoices();
        }
    }

    void selectLocalize() {
        if(localizationMenuFragment != null) {
            localizationMenuFragment.selectLocalize();
        }
    }

    void selectGoToGoal() {
        if(localizationMenuFragment != null) {
            localizationMenuFragment.selectGoToGoal();
        }
    }

    void selectGoToGoalBaseline() {
        if(localizationMenuFragment != null) {
            localizationMenuFragment.selectGoToGoalBaseline();
        }
    }

    void selectGoToGoalForeknowledge() {
        if(localizationMenuFragment != null) {
            localizationMenuFragment.selectGoToGoalForeknowledge();
        }
    }

    void selectGoToGoalAdaptation() {
        if(localizationMenuFragment != null) {
            localizationMenuFragment.selectGoToGoalAdaptation();
        }
    }

    void onLocalizeSelected() {
        Log.d(TAG, "on LocalizeSelected function");
        activity.getScreenMachine().post(ScreenEvent.LOCALIZE_SELECTED); }



    void onGoToGoalSelected() {
        activity.getScreenMachine().post(ScreenEvent.GO_TO_GOAL_SELECTED);
    }


    void onGoToGoalBaselineSelected() {
        activity.getScreenMachine().post(ScreenEvent.GO_TO_GOAL_BASELINE_SELECTED);
    }

    void onGoToGoalForeknowledgeSelected() {
        activity.getScreenMachine().post(ScreenEvent.GO_TO_GOAL_FOREKNOWLEDGE_SELECTED);
    }

    void onGoToGoalAdaptationSelected() {
        activity.getScreenMachine().post(ScreenEvent.GO_TO_GOAL_ADAPTATION_SELECTED);
    }

    void onLocalizeClicked() {
        if (localizationMenuFragment != null) {
            Log.d(TAG, "localizationMenuFragment is not null");
            localizationMenuFragment.disableChoices();
        }
        robot.goToLocalizeBookmark();
    }

    void onGoToGoalClicked() {
        if (localizationMenuFragment != null ) {
            localizationMenuFragment.disableChoices();
        }
        robot.goToGoalBookmark();
    }

    void onGoToGoalBaselineClicked() {
        if (localizationMenuFragment != null ) {
            localizationMenuFragment.disableChoices();
        }
        robot.goToGoalBaselineBookmark();
    }

    void onGoToGoalForeknowledgeClicked() {
        if (localizationMenuFragment != null ) {
            localizationMenuFragment.disableChoices();
        }
        robot.goToGoalForeknowledgeBookmark();
    }

    void onGoToGoalAdaptationClicked() {
        if (localizationMenuFragment != null ) {
            localizationMenuFragment.disableChoices();
        }
        robot.goToGoalAdaptationBookmark();
    }

}
