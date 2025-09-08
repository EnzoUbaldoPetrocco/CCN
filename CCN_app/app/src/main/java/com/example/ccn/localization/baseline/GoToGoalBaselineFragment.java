package com.example.ccn.localization.baseline;

import android.media.MediaPlayer;
import android.os.Bundle;
import android.util.Log;
import android.view.LayoutInflater;
import android.view.View;
import android.view.ViewGroup;
import android.widget.Button;
import android.widget.ImageView;
import android.widget.TextView;

import androidx.annotation.NonNull;
import androidx.annotation.Nullable;
import androidx.annotation.RawRes;
import androidx.fragment.app.Fragment;
import androidx.fragment.app.FragmentActivity;

import com.airbnb.lottie.LottieAnimationView;
import com.example.ccn.R;
import com.example.ccn.core.ClientManager;

import butterknife.BindView;
import butterknife.ButterKnife;
import butterknife.Unbinder;
import io.reactivex.android.schedulers.AndroidSchedulers;
import io.reactivex.disposables.Disposable;
import io.reactivex.schedulers.Schedulers;

/**
 * The go to origin Fragment
 */
public class GoToGoalBaselineFragment extends Fragment {

    @NonNull
    private static final String TAG = "GoToGoalBaselineFragment";

    @Nullable
    private GoToGoalBaselineScreen screen;
    @Nullable
    private GoToGoalBaselineMachine machine;

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

    @Nullable
    private Disposable disposable;

    @Nullable
    private MediaPlayer mediaPlayer;

    @Override
    public View onCreateView(@NonNull LayoutInflater inflater, ViewGroup container,
                             Bundle savedInstanceState) {
        View view = inflater.inflate(R.layout.fragment_go_to_goal_baseline, container, false);
        unbinder = ButterKnife.bind(this, view);

        startGoToButton = view.findViewById(R.id.startGoToButton);
        infoTextView = view.findViewById(R.id.infoTextView);
        warningImage = view.findViewById(R.id.warningImage);
        successImage = view.findViewById(R.id.successImage);
        progressAnimationView = view.findViewById(R.id.progressAnimationView);

        startGoToButton.setOnClickListener(v -> onClickStartGoTo());

        return view;
    }

    @Override
    public void onResume() {
        super.onResume();

        infoTextView.setVisibility(View.INVISIBLE);
        startGoToButton.setVisibility(View.INVISIBLE);
        warningImage.setVisibility(View.INVISIBLE);
        successImage.setVisibility(View.INVISIBLE);
        progressAnimationView.setVisibility(View.INVISIBLE);

        if (machine != null){
            disposable = machine.goToGoalBaselineState()
                    .subscribeOn(Schedulers.io())
                    .observeOn(AndroidSchedulers.mainThread())
                    .subscribe(this::onGoToGoalBaselineStateChanged);
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
            machine.post(GoToGoalBaselineEvent.START_GO_TO_GOAL_BASELINE);
        }
    }

    @NonNull
    static GoToGoalBaselineFragment newInstance(@NonNull GoToGoalBaselineScreen screen, @NonNull GoToGoalBaselineMachine machine) {
        GoToGoalBaselineFragment fragment = new GoToGoalBaselineFragment();
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

    private void onGoToGoalBaselineStateChanged(@NonNull GoToGoalBaselineState goToGoalBaselineState){
        Log.d(TAG, "onGoToOriginStateChanged: " + goToGoalBaselineState);

        switch (goToGoalBaselineState) {
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
                    screen.onGoToGoalBaselineEnd();
                }
                break;
        }
    }
}
