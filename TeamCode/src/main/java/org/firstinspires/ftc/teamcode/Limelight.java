package org.firstinspires.ftc.teamcode;

// Imports for the Lazar full-panels library
//import com.lazar.fullpanels.FtcDashboard;
//import com.lazar.fullpanels.TelemetryPacket;

import java.net.HttpURLConnection;
import java.net.URL;

/**
 * A simple and robust class to interface with a Limelight camera.
 * This class uses HTTP GET requests to set Limelight properties.
 *
 * It is recommended to create one instance of this class in your OpMode.
 * Network operations are run on a separate thread to prevent blocking the main robot loop.
 */
public class Limelight {

    private final String limelightName;
//    private final FtcDashboard dashboard;

    /**
     * Default constructor. Assumes the Limelight's hostname is "limelight".
     */
    public Limelight() {
        this("limelight"); // Calls the other constructor with the default name
    }

    /**
     * Constructor to specify a custom Limelight hostname.
     * @param name The hostname of the Limelight (e.g., "limelight-drive").
     */
    public Limelight(String name) {
        this.limelightName = name;
        // This will get the instance of the fullpanels dashboard
//        this.dashboard = FtcDashboard.getInstance();
    }

    /**
     * Sends a GET request to the Limelight to set a parameter on a separate thread.
     * @param param The name of the parameter to set (e.g., "ledMode").
     * @param val The value to set the parameter to.
     */
    private void setParam(String param, int val) {
        // Run network operations on a new thread to avoid blocking the OpMode loop.
        new Thread(() -> {
            try {
                // Construct the URL for the GET request
                URL url = new URL("http://" + limelightName + ".local:5801/?" + param + "=" + val);
                HttpURLConnection connection = (HttpURLConnection) url.openConnection();
                connection.setRequestMethod("GET");
                // Set short timeouts to prevent long waits on connection failure
                connection.setConnectTimeout(500); // Increased timeout slightly for reliability
                connection.setReadTimeout(500);

                int responseCode = connection.getResponseCode();
                // Send telemetry feedback only if a dashboard instance is available
//                if (dashboard != null) {
//                    TelemetryPacket packet = new TelemetryPacket();
//                    packet.put("Limelight " + param, "Sent: " + val + " | Response: " + responseCode);
//                    dashboard.sendTelemetryPacket(packet);
//                }

                connection.disconnect();

            } catch (Exception e) {
                // Log any errors to the fullpanels dashboard for debugging
//                if (dashboard != null) {
//                    TelemetryPacket packet = new TelemetryPacket();
//                    packet.put("Limelight Connection Error", e.getMessage());
//                    dashboard.sendTelemetryPacket(packet);
//                }
            }
        }).start(); // Start the new thread
    }

    /**
     * Sets the Limelight's LED mode.
     * 0: Use the LED mode set in the current pipeline.
     * 1: Force LEDs off.
     * 2: Force LEDs to blink.
     * 3: Force LEDs on.
     * @param mode The desired LED mode (0-3).
     */
    public void setLedMode(int mode) {
        if (mode < 0 || mode > 3) return; // Basic validation
        setParam("ledMode", mode);
    }

    /**
     * Sets the Limelight's camera mode.
     * 0: Vision Processor mode.
     * 1: Driver Camera mode (increases exposure, disables vision processing).
     * @param mode The desired camera mode (0-1).
     */
    public void setCameraMode(int mode) {
        if (mode < 0 || mode > 1) return;
        setParam("cameraMode", mode);
    }

    /**
     * Sets the active pipeline for the Limelight.
     * Pipelines are numbered 0 through 9.
     * @param pipeline The pipeline index to set (0-9).
     */
    public void setPipeline(int pipeline) {
        if (pipeline < 0 || pipeline > 9) return;
        setParam("pipeline", pipeline);
    }

    /**
     * Sets the camera's exposure value.
     * Common values range from 0-100 for tracking.
     * @param exposure The exposure value to set.
     */
    public void setExposure(int exposure) {
        // The parameter name for exposure can vary. "cam-exposure" is a common one.
        // If this doesn't work, check your Limelight's documentation for the correct API parameter.
        setParam("cam-exposure", exposure);
    }
}
