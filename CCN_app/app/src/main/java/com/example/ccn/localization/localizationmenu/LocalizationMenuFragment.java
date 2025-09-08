package com.example.ccn.localization.localizationmenu;

import android.os.Bundle;
import android.provider.MediaStore;
import android.util.Log;
import android.view.LayoutInflater;
import android.view.View;
import android.view.ViewGroup;
import android.widget.RadioButton;

import com.example.ccn.R;
import com.example.ccn.mapping.MappingEvent;

import androidx.annotation.NonNull;
import androidx.annotation.Nullable;
import androidx.fragment.app.Fragment;
import androidx.fragment.app.FragmentActivity;
import butterknife.BindView;
import butterknife.ButterKnife;
import butterknife.OnClick;
import butterknife.Unbinder;

/**
 * The localization menu Fragment
 */
public class LocalizationMenuFragment extends Fragment {

    @NonNull
    private static final String TAG = "LocalizationMenuFragment";

    @Nullable
    private LocalizationMenuScreen screen;

    @Nullable
    private Unbinder unbinder;

    RadioButton localizeButton;
    RadioButton goToGoalButton;
    RadioButton goToGoalBaselineButton;
    RadioButton goToGoalForeknowledgeButton;
    RadioButton goToGoalAdaptationButton;

    @Override
    public View onCreateView(@NonNull LayoutInflater inflater, ViewGroup container,
                             Bundle savedInStanceState) {
        View view = inflater.inflate(R.layout.fragment_localization_menu, container, false);
        unbinder = ButterKnife.bind(this, view);
        localizeButton = view.findViewById(R.id.localizeButton);
        goToGoalButton = view.findViewById(R.id.goToGoalButton);
        goToGoalBaselineButton = view.findViewById(R.id.goToGoalButtonBaseline);
        goToGoalForeknowledgeButton = view.findViewById(R.id.goToGoalButtonForeknowledge);
        goToGoalAdaptationButton = view.findViewById(R.id.goToGoalButtonAdaptation);

        localizeButton.setOnClickListener(v -> onClickLocalizeButton());
        goToGoalButton.setOnClickListener(v -> onClickGoGoalButton());
        goToGoalBaselineButton.setOnClickListener(v -> onClickGoGoalBaselineButton());
        goToGoalForeknowledgeButton.setOnClickListener(v -> onClickGoGoalForeknowledgeButton());
        goToGoalAdaptationButton.setOnClickListener(v -> onClickGoGoalAdaptationButton());

        return view;
    }

    @Override
    public void onResume() {
        super.onResume();
        localizeButton.setChecked(false);
        goToGoalButton.setChecked(false);
        goToGoalBaselineButton.setChecked(false);
        goToGoalForeknowledgeButton.setChecked(false);
        goToGoalAdaptationButton.setChecked(false);
    }

    @Override
    public void onDestroyView() {
        if(unbinder!=null) {
            unbinder.unbind();
        }
        super.onDestroyView();
    }

    public void onClickLocalizeButton() {
        Log.d(TAG, "Clicked localizeButton");
        if (screen != null) {
            Log.d(TAG, "Screen is not null");
            screen.onLocalizeClicked();
        }
    }

    public void onClickGoGoalButton() {
        Log.d(TAG, "Clicked goToGoalButton");
        if (screen != null) {
            screen.onGoToGoalClicked();
        }
    }

    public void onClickGoGoalBaselineButton() {
        Log.d(TAG, "Clicked goToGoalBaselineButton");
        if (screen != null) {
            screen.onGoToGoalBaselineClicked();
        }
    }

    public void onClickGoGoalForeknowledgeButton() {
        Log.d(TAG, "Clicked goToGoalForeknowledgeButton");
        if (screen != null) {
            screen.onGoToGoalForeknowledgeClicked();
        }
    }

    public void onClickGoGoalAdaptationButton() {
        Log.d(TAG, "Clicked goToGoalAdaptationButton");
        if (screen != null) {
            screen.onGoToGoalAdaptationClicked();
        }
    }

    @NonNull
    static LocalizationMenuFragment newInstance(@NonNull LocalizationMenuScreen screen) {
        LocalizationMenuFragment fragment = new LocalizationMenuFragment();
        fragment.screen = screen;
        return fragment;
    }


    void disableChoices() {
        runOnUiThread(() -> {
            localizeButton.setEnabled(false);
            goToGoalButton.setEnabled(false);
            goToGoalBaselineButton.setEnabled(false);
            goToGoalForeknowledgeButton.setEnabled(false);
            goToGoalAdaptationButton.setEnabled(false);
        });
    }

    void selectLocalize() {runOnUiThread(() -> localizeButton.setChecked(true)); }
    void selectGoToGoal() {runOnUiThread(() -> goToGoalButton.setChecked(true)); }
    void selectGoToGoalBaseline() {runOnUiThread(() -> goToGoalBaselineButton.setChecked(true)); }
    void selectGoToGoalForeknowledge() {runOnUiThread(() -> goToGoalForeknowledgeButton.setChecked(true)); }
    void selectGoToGoalAdaptation() {runOnUiThread(() -> goToGoalAdaptationButton.setChecked(true)); }


    private void runOnUiThread(@NonNull Runnable runnable) {
        FragmentActivity activity = getActivity();
        if (activity != null) {
            activity.runOnUiThread(runnable);
        }
    }

}
