package com.example.nnc;

import androidx.appcompat.app.AppCompatActivity;
import android.os.Bundle;
import android.util.Log;

import com.aldebaran.qi.Future;
import com.aldebaran.qi.sdk.QiContext;
import com.aldebaran.qi.sdk.QiSDK;
import com.aldebaran.qi.sdk.RobotLifecycleCallbacks;
import com.aldebaran.qi.sdk.builder.SayBuilder;
import com.aldebaran.qi.sdk.builder.TakePictureBuilder;
import com.aldebaran.qi.sdk.design.activity.RobotActivity;
import com.aldebaran.qi.sdk.object.camera.TakePicture;
import com.aldebaran.qi.sdk.object.conversation.Phrase;
import com.aldebaran.qi.sdk.object.conversation.Say;
import com.aldebaran.qi.sdk.object.image.EncodedImage;
import com.aldebaran.qi.sdk.object.image.EncodedImageHandle;
import com.aldebaran.qi.sdk.object.image.TimestampedImageHandle;
import com.aldebaran.qi.sdk.object.locale.Language;
import com.aldebaran.qi.sdk.object.locale.Locale;
import com.aldebaran.qi.sdk.object.locale.Region;

import org.json.JSONException;
import org.json.JSONObject;

import java.io.*;
import java.net.HttpURLConnection;
import java.net.URL;
import java.util.Timer;
import java.util.TimerTask;

class ImageUploader {

    public static JSONObject uploadImage(String targetURL, EncodedImage encodedImage, int counter) throws IOException {
        byte[] imageData = encodedImage.getData().array(); // Extract image data as a byte array
        String boundary = "----WebKitFormBoundary" + System.currentTimeMillis(); // Unique boundary
        String fileName = "image_" + counter + ".jpg"; // Generate a filename using the counter

        HttpURLConnection connection = (HttpURLConnection) new URL(targetURL).openConnection();
        connection.setDoOutput(true);
        connection.setRequestMethod("POST");
        connection.setRequestProperty("Content-Type", "multipart/form-data; boundary=" + boundary);

        try (OutputStream outputStream = connection.getOutputStream();
             PrintWriter writer = new PrintWriter(new OutputStreamWriter(outputStream, "UTF-8"), true)) {

            // Add file part
            writer.append("--").append(boundary).append("\r\n");
            writer.append("Content-Disposition: form-data; name=\"image\"; filename=\"")
                    .append(fileName).append("\"\r\n");
            writer.append("Content-Type: image/jpeg\r\n\r\n");
            writer.flush();

            // Write the image data to the output stream
            outputStream.write(imageData);
            outputStream.flush();

            // End of multipart/form-data
            writer.append("\r\n").append("--").append(boundary).append("--").append("\r\n");
            writer.flush();
        }


        try (BufferedReader reader = new BufferedReader(new InputStreamReader(connection.getInputStream()))) {
            StringBuilder responseBuilder = new StringBuilder();
            String line;

            // Read the entire response into a single string
            while ((line = reader.readLine()) != null) {
                responseBuilder.append(line);
            }

            // Parse the JSON response
            String jsonResponse = responseBuilder.toString();
            JSONObject jsonObject = new JSONObject(jsonResponse);

            return jsonObject;
        } catch (JSONException e) {
            e.printStackTrace();
            return null;
        }


    }
}

    public class MainActivity extends RobotActivity implements RobotLifecycleCallbacks {

    int counter = 0;
    String targetURL = "http://192.168.171.113:5000/predict"; // Flask server endpoint
    String currentCommand = null;
    double prediction = 0.0;
    private static final String TAG = "RobotActivity";

    // Navigation parameters
        double x=0.0;
        double y=0.0;
        double xDot=0.0;
        double yDot=0.0;
        double theta=0.0;
        double thetaDot=0.0;


        @Override
    protected void onCreate(Bundle savedInstanceState) {
        super.onCreate(savedInstanceState);
        // Register the RobotLifecycleCallbacks to this Activity.
        QiSDK.register(this, this);

    }

    @Override
    protected void onDestroy() {
        // Unregister the RobotLifecycleCallbacks for this Activity.
        QiSDK.unregister(this, this);
        super.onDestroy();
    }

    public void takePicture() {

    }

    @Override
    public void onRobotFocusGained(QiContext qiContext) {
        ImageUploader img_uploader = new ImageUploader();
        Timer timer = new Timer();

        TimerTask sendInfo2Server = new TimerTask()  {
            @Override
            public void run() {

                // Build the action.
                TakePicture takePicture = TakePictureBuilder.with(qiContext).build();

                // Run the action synchronously
                TimestampedImageHandle result = takePicture.run();

                // Retrieve the encoded image
                EncodedImageHandle encodedImageHandle = result.getImage(); // Fixed to call on result
                EncodedImage encodedImage = encodedImageHandle.getValue(); // Retrieve EncodedImage value

                try {

                    JSONObject jsonObject =  img_uploader.uploadImage(targetURL, encodedImage, counter);

                    // Extract specific fields from the JSON object
                    if (jsonObject.has("prediction")) {
                        prediction = (double) jsonObject.get("prediction");
                        Log.d(TAG, "Prediction: " + prediction);

                    }

                    if (jsonObject.has("current_command")) {
                        currentCommand = jsonObject.getString("current_command");
                        Log.d(TAG, "Current Command: " + currentCommand);
                    }
                    counter++;

                } catch (Exception e) {
                    e.printStackTrace();
                }
            }

        };


        timer.scheduleAtFixedRate(sendInfo2Server,0, 3000);
        // Blocking the thread to keep the focus
        while (true) {
            try {
                Thread.sleep(3000); // Prevent busy-waiting
            } catch (InterruptedException e) {
                Log.e(TAG, "Interrupted: " + e.getMessage());
                break;
            }
        }

        // Cancel the timer when exiting (optional)
        timer.cancel();

    }

    @Override
    public void onRobotFocusLost() {
        // The robot focus is lost.
    }

    @Override
    public void onRobotFocusRefused(String reason) {
        // The robot focus is refused.
    }

}
