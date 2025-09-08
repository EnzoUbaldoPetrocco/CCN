package com.example.ccn.brute_services;

import android.media.MediaPlayer;
import android.os.Bundle;
import android.util.Log;
import android.view.View;
import android.widget.Button;
import android.widget.ImageView;
import android.widget.LinearLayout;
import android.widget.Toast;

import androidx.annotation.NonNull;
import androidx.annotation.Nullable;
import androidx.annotation.RawRes;

import com.aldebaran.qi.sdk.QiContext;
import com.aldebaran.qi.sdk.QiSDK;
import com.aldebaran.qi.sdk.design.activity.RobotActivity;
import com.aldebaran.qi.sdk.object.actuation.ExplorationMap;
import com.example.ccn.core.ClientManager;

import org.json.JSONObject;

import java.util.ArrayList;
import java.util.List;

import io.reactivex.android.schedulers.AndroidSchedulers;
import io.reactivex.disposables.Disposable;
import io.reactivex.schedulers.Schedulers;
import com.example.ccn.R;


/**
 * The Menu Services Activity
 */
public class MenuServicesActivity extends RobotActivity {

    @NonNull
    private static final String TAG = "ServiceMenuActivity";

    @NonNull
    private final MenuServicesRobot robot = new MenuServicesRobot();

    @Nullable
    private MediaPlayer mediaPlayer;



    private class MenuItem {
        String label;
        Runnable action;

        MenuItem(String label, Runnable action) {
            this.label = label;
            this.action = action;
        }
    }

    private final List<MenuItem> menuItems = new ArrayList<>();

    @Override
    protected void onCreate(Bundle savedInstanceState) {
        super.onCreate(savedInstanceState);

        setContentView(R.layout.activity_menu_services);

        LinearLayout container = findViewById(R.id.menu_container);

        // Populate menu items
        String getRobotPositionLabel = "Get Robot Position";
        MenuItem getRobotPositionItem  = new MenuItem(getRobotPositionLabel, this::getRobotPosition);
        menuItems.add(getRobotPositionItem);

        String getGoalPositionLabel = "Get Goal Position";
        MenuItem getGoalPositionItem  = new MenuItem(getGoalPositionLabel, this::getGoalPosition);
        menuItems.add(getGoalPositionItem);

        String postRobotPositionLabel = "Post Robot Position";
        MenuItem postRobotPositionItem  = new MenuItem(postRobotPositionLabel, this::postRobotPosition);
        menuItems.add(postRobotPositionItem);

        String postGoalPositionLabel = "Post Goal Position";
        MenuItem postGoalPositionItem = new MenuItem(postGoalPositionLabel, this::postGoalPosition);
        menuItems.add(postGoalPositionItem);

        String sendMapLabel = "Send Map to Server";
        MenuItem sendMapItem = new MenuItem(sendMapLabel, this::postMap);
        menuItems.add(sendMapItem);

        String getMapLabel = "Load Map from Server";
        MenuItem getMapItem = new MenuItem(getMapLabel, this::getMap);
        menuItems.add(getMapItem);

        String getNavigationPathLabel = "Get Navigation Path";
        MenuItem getNavigationPathItem = new MenuItem(getNavigationPathLabel, this::getNavigationPath);
        menuItems.add(getNavigationPathItem);

        String getNavigationPointLabel = "Get Navigation Point";
        MenuItem getNavigationPointItem = new MenuItem(getNavigationPointLabel, this::getNavigationPoint);
        menuItems.add(getNavigationPointItem);

        String getNavigationPathWithPlotLabel = "Get Path with Plots";
        MenuItem getNavigationPathWithPlotItem = new MenuItem(getNavigationPathWithPlotLabel, this::getNavigationWithPlot);
        menuItems.add(getNavigationPathWithPlotItem);

        String getRobotInMapLabel = "Visualize Robot in Map";
        MenuItem getRobotInMapItem = new MenuItem(getRobotInMapLabel, this::getRobotInMap);
        menuItems.add(getRobotInMapItem);

        String postCameraInfoLabel = "Send Camera Info";
        MenuItem postCameraInfoItem = new MenuItem(postCameraInfoLabel, this::postCameraInfo);
        menuItems.add(postCameraInfoItem);

        String postDCameraInfoLabel = "Send Depth Camera Info";
        MenuItem postDCameraInfoItem = new MenuItem(postDCameraInfoLabel, this::postDCameraInfo);
        menuItems.add(postDCameraInfoItem);

        String getRobotPositionCameraLabel = "Get Robot position using Camera Info";
        MenuItem getRobotPositionCameraItem = new MenuItem(getRobotPositionCameraLabel, this::getRobotPositionCamera);
        menuItems.add(getRobotPositionCameraItem);

        // Create buttons dynamically
        for (MenuItem item : menuItems) {
            Button button = new Button(this);
            button.setText(item.label);
            button.setOnClickListener(v -> {
                Toast.makeText(MenuServicesActivity.this, "Calling " + item.label, Toast.LENGTH_SHORT).show();
                try {
                    item.action.run();
                } catch (Exception e) {
                    Toast.makeText(MenuServicesActivity.this, "Error: " + e.getMessage(), Toast.LENGTH_SHORT).show();
                }
            });
            container.addView(button);
        }

        ImageView closeButton = findViewById(R.id.closeButton);
        ImageView backButton = findViewById(R.id.backButton);

        closeButton.setOnClickListener(v -> finishAffinity());
        backButton.setOnClickListener(v -> onBackPressed());

        QiSDK.register(this, robot);
    }

    protected void getRobotPosition(){
        robot.getRobotPosition();
    }

    protected void getGoalPosition(){
        robot.getGoalPosition();
    }

    void postRobotPosition(){
        robot.postRobotPosition();
    }

    void postGoalPosition(){
        try {
            JSONObject goal = new JSONObject();
            goal.put("x", 1.0);
            goal.put("y", 1.0);
            goal.put("theta", 0.0);
            robot.postGoalPosition(goal);
        } catch (Exception e) {
            e.printStackTrace();
        }
    }

    void postMap(){
        robot.postMap();
    }

    void getMap(){
        robot.getMap();
    }

    void getNavigationPath(){
        robot.getNavigationPath();
    }

    void getNavigationPoint(){
        robot.getNavigationPoint();
    }

    void getNavigationWithPlot(){
        robot.getNavigationWithPlot();
    }

    void getRobotInMap(){
        robot.getRobotInMap();
    }

    void postCameraInfo(){
        robot.postCameraInfo();
    }
    void postDCameraInfo(){
        robot.postDCameraInfo();
    }

    void getRobotPositionCamera() {robot.getRobotPositionCamera(); }

        @Override
        protected void onResume() {
            super.onResume();

        }

        @Override
        protected void onPause() {

            if(mediaPlayer != null) {
                mediaPlayer.release();
                mediaPlayer = null;
            }

            super.onPause();
        }

        @Override
        protected void onDestroy() {
            QiSDK.unregister(this, robot);
            super.onDestroy();
        }


        private void playSound(@RawRes int soundResId, boolean playInLoop){
            if(mediaPlayer != null){
                mediaPlayer.release();
            }

            mediaPlayer = MediaPlayer.create(this, soundResId);
            mediaPlayer.setLooping(playInLoop);
            mediaPlayer.start();
        }

}
