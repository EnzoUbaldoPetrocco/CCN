package com.example.ccn.localization.localizationmenu;

import android.util.Log;

import com.aldebaran.qi.Future;
import com.aldebaran.qi.sdk.QiContext;
import com.aldebaran.qi.sdk.builder.ChatBuilder;
import com.aldebaran.qi.sdk.builder.QiChatbotBuilder;
import com.aldebaran.qi.sdk.builder.TopicBuilder;
import com.aldebaran.qi.sdk.object.conversation.AutonomousReactionImportance;
import com.aldebaran.qi.sdk.object.conversation.AutonomousReactionValidity;
import com.aldebaran.qi.sdk.object.conversation.Bookmark;
import com.aldebaran.qi.sdk.object.conversation.BookmarkStatus;
import com.aldebaran.qi.sdk.object.conversation.Chat;
import com.aldebaran.qi.sdk.object.conversation.QiChatVariable;
import com.aldebaran.qi.sdk.object.conversation.QiChatbot;
import com.aldebaran.qi.sdk.object.conversation.Topic;

import com.example.ccn.R;
import com.example.ccn.localization.LocalizeManager;
import com.example.ccn.localization.Robot;
import com.example.ccn.utils.FutureCancellations;

import java.util.Map;
import java.util.concurrent.TimeUnit;
import java.util.concurrent.atomic.AtomicBoolean;

import androidx.annotation.NonNull;
import androidx.annotation.Nullable;
import io.reactivex.Single;
import io.reactivex.disposables.Disposable;
import io.reactivex.schedulers.Schedulers;


/**
 * The robot for {@link}
 */
public class LocalizationMenuRobot implements Robot {

    @NonNull
    private static final String TAG = "LocalizationMenuRobot";

    @NonNull
    private final LocalizationMenuScreen screen;
    @NonNull
    private final LocalizeManager localizeManager;

    @Nullable
    private Disposable timerDisposable;

    private Future<Void> discussion;

    LocalizationMenuRobot(@NonNull LocalizationMenuScreen screen, @NonNull LocalizeManager localizeManager){
        this.screen = screen;
        this.localizeManager = localizeManager;
    }

    @NonNull
    @Override
    public Future<Void> stop() {
        stopTimer();
        return FutureCancellations.cancel(discussion);
    }

    void goToLocalizeBookmark() {
        screen.onLocalizeSelected();
    }


    void goToGoalBookmark() {
        screen.onGoToGoalSelected();
    }

    void goToGoalBaselineBookmark() {
        screen.onGoToGoalBaselineSelected();
    }

    void goToGoalForeknowledgeBookmark() {
        screen.onGoToGoalForeknowledgeSelected();
    }

    void goToGoalAdaptationBookmark() {
        screen.onGoToGoalAdaptationSelected();
    }

    private void startTimer() {
        timerDisposable = Single.timer(5, TimeUnit.SECONDS)
                .observeOn(Schedulers.io())
                .subscribeOn(Schedulers.io())
                .subscribe(ignored -> {
                    stopTimer();
                });
    }

    private void stopTimer() {
        if (timerDisposable != null && !timerDisposable.isDisposed()) {
            timerDisposable.dispose();
        }
    }
}
