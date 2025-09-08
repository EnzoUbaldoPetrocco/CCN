package com.example.ccn.localization.localize;

import android.util.Log;

import com.aldebaran.qi.Future;
import com.aldebaran.qi.sdk.QiContext;
import com.example.ccn.R;
import com.example.ccn.localization.LocalizationActivity;
import com.example.ccn.localization.LocalizeManager;
import com.example.ccn.localization.Screen;
import com.example.ccn.localization.ScreenEvent;

import androidx.annotation.NonNull;

/**
 * Localize Screen
 */
public class LocalizeScreen implements Screen {

    @NonNull
    private static final String TAG = "LocalizeScreen";

    @NonNull
    private final LocalizationActivity activity;
    @NonNull
    private final LocalizeRobot robot;
    @NonNull
    private final LocalizeMachine machine;

    public LocalizeScreen(@NonNull LocalizationActivity activity, @NonNull LocalizeManager localizeManager){
        this.activity = activity;
        this.machine = new LocalizeMachine(localizeManager);
        this.robot = new LocalizeRobot(machine, localizeManager);
    }

    @Override
    public void start(@NonNull QiContext qiContext){
        activity.setNavigationTitle(R.string.localize_title);

        LocalizeFragment fragment = LocalizeFragment.newInstance(this, machine);
        activity.showFragment(fragment);
        robot.start(qiContext);
    }

    @NonNull
    @Override
    public Future<Void> stop() {return robot.stop(); }

    void onLocalizeEnd() {
        Log.d(TAG, "LocalizationEnded");
        activity.getScreenMachine().post(ScreenEvent.LOCALIZE_END);
    }
}














