package com.example.ccn.localization.foreknowledge;

import android.media.MediaPlayer;
import android.os.Bundle;
import android.util.Log;
import android.view.LayoutInflater;
import android.view.View;
import android.view.ViewGroup;
import android.widget.Button;
import android.widget.ImageView;
import android.widget.TextView;
import android.widget.Toast;

import androidx.annotation.NonNull;
import androidx.annotation.Nullable;
import androidx.annotation.RawRes;
import androidx.fragment.app.Fragment;
import androidx.fragment.app.FragmentActivity;

import com.airbnb.lottie.LottieAnimationView;
import com.example.ccn.R;
import com.example.ccn.core.ClientManager;

import java.util.Locale;

import butterknife.BindView;
import butterknife.ButterKnife;
import butterknife.Unbinder;
import io.reactivex.android.schedulers.AndroidSchedulers;
import io.reactivex.disposables.Disposable;
import io.reactivex.schedulers.Schedulers;

/**
 * The go to origin Fragment
 */
public class GoToGoalForeknowledgeFragment extends Fragment {

    @NonNull
    private static final String TAG = "GoToGoalForeknowledgeFragment";

    @Nullable
    private GoToGoalForeknowledgeScreen screen;
    @Nullable
    private GoToGoalForeknowledgeMachine machine;

    @Nullable
    private Unbinder unbinder;

    @BindView(R.id.startGoToButton)
    Button startGoToButton;

    @BindView(R.id.infoTextView)
    TextView infoTextView;

    @BindView(R.id.warningImage)
    ImageView warningImage;

    @BindView(R.id.successImage)
    ImageView successImage;

    @BindView(R.id.progressAnimationView)
    LottieAnimationView progressAnimationView;

    @BindView(R.id.buttonItalian)
    Button buttonItalian;

    @BindView(R.id.buttonGerman)
    Button buttonGerman;

    private String culture;


    @Nullable
    private Disposable disposable;

    @Nullable
    private MediaPlayer mediaPlayer;


    @Override
    public View onCreateView(@NonNull LayoutInflater inflater, ViewGroup container,
                             Bundle savedInstanceState) {
        View view = inflater.inflate(R.layout.fragment_go_to_goal_foreknowledge, container, false);
        unbinder = ButterKnife.bind(this, view);

        startGoToButton = view.findViewById(R.id.startGoToButton);
        infoTextView = view.findViewById(R.id.infoTextView);
        warningImage = view.findViewById(R.id.warningImage);
        successImage = view.findViewById(R.id.successImage);
        progressAnimationView = view.findViewById(R.id.progressAnimationView);
        buttonItalian = view.findViewById(R.id.buttonItalian);
        buttonGerman = view.findViewById(R.id.buttonGerman);

        startGoToButton.setOnClickListener(v -> onClickStartGoTo());


        buttonItalian.setOnClickListener(v -> {
            culture = "Italian";
            this.screen.robot.currentLocale = Locale.ITALIAN;
            this.screen.robot.culture = culture;
            Toast.makeText(getContext(), "Culture set to Italian", Toast.LENGTH_SHORT).show();
            hideCultureButtons();
            startGoToButton.setVisibility(View.VISIBLE);
        });

        buttonGerman.setOnClickListener(v -> {
            culture = "German";
            this.screen.robot.currentLocale = Locale.GERMAN;
            this.screen.robot.culture = culture;
            Toast.makeText(getContext(), "Culture set to German", Toast.LENGTH_SHORT).show();
            hideCultureButtons();
            startGoToButton.setVisibility(View.VISIBLE);
        });


        return view;
    }

    private void hideCultureButtons() {
        buttonItalian.setVisibility(View.GONE);
        buttonGerman.setVisibility(View.GONE);
    }


    @Override
    public void onResume() {
        super.onResume();

        culture = null;
        buttonItalian.setVisibility(View.VISIBLE);
        buttonGerman.setVisibility(View.VISIBLE);
        startGoToButton.setVisibility(View.INVISIBLE);
        infoTextView.setVisibility(View.INVISIBLE);
        warningImage.setVisibility(View.INVISIBLE);
        successImage.setVisibility(View.INVISIBLE);
        progressAnimationView.setVisibility(View.INVISIBLE);


        if (machine != null){
            disposable = machine.goToGoalForeknowledgeState()
                    .subscribeOn(Schedulers.io())
                    .observeOn(AndroidSchedulers.mainThread())
                    .subscribe(this::onGoToGoalForeknowledgeStateChanged);
        }
    }

    @Override
    public void onDestroyView() {
        if(unbinder!=null){
            unbinder.unbind();
        }
        super.onDestroyView();
    }

    public void onClickStartGoTo() {
        if (machine!=null) {
            machine.post(GoToGoalForeknowledgeEvent.START_GO_TO_GOAL_FOREKNOWLEDGE);
        }
    }



    @NonNull
    static GoToGoalForeknowledgeFragment newInstance(@NonNull GoToGoalForeknowledgeScreen screen, @NonNull GoToGoalForeknowledgeMachine machine) {
        GoToGoalForeknowledgeFragment fragment = new GoToGoalForeknowledgeFragment();
        fragment.screen = screen;
        fragment.machine = machine;
        return fragment;
    }

    private void playSound(@RawRes int soundResId) {
        if(mediaPlayer!=null){
            mediaPlayer.release();
        }

        FragmentActivity activity = getActivity();
        if(activity!=null){
            mediaPlayer = MediaPlayer.create(activity, soundResId);
            mediaPlayer.start();
        }
    }

    private void onGoToGoalForeknowledgeStateChanged(@NonNull GoToGoalForeknowledgeState goToGoalForeknowledgeState){
        Log.d(TAG, "onGoToOriginStateChanged: " + goToGoalForeknowledgeState);

        switch (goToGoalForeknowledgeState) {
            case IDLE:
                infoTextView.setVisibility(View.INVISIBLE);
                startGoToButton.setVisibility(View.INVISIBLE);
                warningImage.setVisibility(View.GONE);
                successImage.setVisibility(View.INVISIBLE);
                progressAnimationView.setVisibility(View.INVISIBLE);
                break;
            case BRIEFING:
                infoTextView.setVisibility(View.VISIBLE);
                startGoToButton.setVisibility(View.VISIBLE);
                warningImage.setVisibility(View.GONE);
                successImage.setVisibility(View.INVISIBLE);
                infoTextView.setText(R.string.go_to_goal_briefing_text);
                progressAnimationView.setVisibility(View.INVISIBLE);
                break;
            case MOVING:
                infoTextView.setVisibility(View.VISIBLE);
                startGoToButton.setVisibility(View.INVISIBLE);
                warningImage.setVisibility(View.GONE);
                successImage.setVisibility(View.INVISIBLE);
                infoTextView.setText(R.string.go_to_origin_moving_text);
                progressAnimationView.setVisibility(View.VISIBLE);
                break;
            case ERROR:
                infoTextView.setVisibility(View.VISIBLE);
                startGoToButton.setVisibility(View.VISIBLE);
                warningImage.setVisibility(View.VISIBLE);
                successImage.setVisibility(View.INVISIBLE);
                infoTextView.setText(R.string.error_text);
                progressAnimationView.setVisibility(View.INVISIBLE);
                playSound(R.raw.error);
                break;
            case SUCCESS:
                infoTextView.setVisibility(View.VISIBLE);
                startGoToButton.setVisibility(View.INVISIBLE);
                warningImage.setVisibility(View.GONE);
                successImage.setVisibility(View.VISIBLE);
                infoTextView.setText(R.string.success_text);
                progressAnimationView.setVisibility(View.INVISIBLE);
                playSound(R.raw.success);
                break;
            case END:
                if(screen!=null){
                    screen.onGoToGoalForeknowledgeEnd();
                }
                break;
        }
    }
}
