package org.firstinspires.ftc.teamcode.pedroPathing.Pipelines;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;

public class Limelight_Pipeline {

    public static Limelight3A limelight;
    private static boolean isInitialized = false;

    /**
     * Initialize the Limelight in AprilTag mode (pipeline 0)
     */
    public static void initLimelight(OpMode opMode) {
        try {
            if (!isInitialized) {
                limelight = opMode.hardwareMap.get(Limelight3A.class, "limelight");
                isInitialized = true;
            }
            
            // // Stop first to reset
            limelight.stop();
            
            // Set to AprilTag pipeline (pipeline 0)
            limelight.pipelineSwitch(0);
            
            // Set polling mode to enable camera streaming
            limelight.setPollRateHz(100); // 100Hz polling for smooth tracking
            
            // Start the Limelight
            limelight.start();
            
            // Give it a moment to initialize
            try { Thread.sleep(100); } catch (InterruptedException e) {}
            
            // Force an initial poll
            LLResult testResult = limelight.getLatestResult();
            
            opMode.telemetry.addData("Limelight", "Initialized - AprilTag Mode");
            opMode.telemetry.addData("LL Streaming", "Enabled at 100Hz");
            opMode.telemetry.addData("LL Status", testResult != null ? "Connected" : "No Data");
            opMode.telemetry.update();
        } catch (Exception e) {
            opMode.telemetry.addData("LL ERROR", e.getMessage());
            opMode.telemetry.update();
        }
    }

    /**
     * Get the latest result from the Limelight
     */
    public static LLResult getLatestResult() {
        if (limelight != null) {
            return limelight.getLatestResult();
        }
        return null;
    }

    /**
     * Check if a target is detected
     */
    public static boolean hasTarget() {
        LLResult result = getLatestResult();
        return result != null && result.isValid();
    }

    /**
     * Get the horizontal offset (tx) of the target
     */
    public static double getTargetX() {
        LLResult result = getLatestResult();
        if (result != null && result.isValid() && result.getFiducialResults() != null && !result.getFiducialResults().isEmpty()) {
            return result.getFiducialResults().get(0).getTargetXDegrees();
        }
        return 0.0;
    }

    /**
     * Get the vertical offset (ty) of the target
     */
    public static double getTargetY() {
        LLResult result = getLatestResult();
        if (result != null && result.isValid() && result.getFiducialResults() != null && !result.getFiducialResults().isEmpty()) {
            return result.getFiducialResults().get(0).getTargetYDegrees();
        }
        return 0.0;
    }

    /**
     * Check if blue goal (ID 20) is detected
     */
    public static boolean hasBlueGoal() {
        LLResult result = getLatestResult();
        if (result != null && result.isValid() && result.getFiducialResults() != null) {
            for (int i = 0; i < result.getFiducialResults().size(); i++) {
                if ((int) result.getFiducialResults().get(i).getFiducialId() == 20) {
                    return true;
                }
            }
        }
        return false;
    }

    /**
     * Check if red goal (ID 24) is detected
     */
    public static boolean hasRedGoal() {
        LLResult result = getLatestResult();
        if (result != null && result.isValid() && result.getFiducialResults() != null) {
            for (int i = 0; i < result.getFiducialResults().size(); i++) {
                if ((int) result.getFiducialResults().get(i).getFiducialId() == 24) {
                    return true;
                }
            }
        }
        return false;
    }

    /**
     * Get horizontal offset (tx) for blue goal (ID 20)
     */
    public static double getBlueGoalX() {
        LLResult result = getLatestResult();
        if (result != null && result.isValid() && result.getFiducialResults() != null) {
            for (int i = 0; i < result.getFiducialResults().size(); i++) {
                if ((int) result.getFiducialResults().get(i).getFiducialId() == 20) {
                    return result.getFiducialResults().get(i).getTargetXDegrees();
                }
            }
        }
        return 0.0;
    }

    /**
     * Get vertical offset (ty) for blue goal (ID 20)
     */
    public static double getBlueGoalY() {
        LLResult result = getLatestResult();
        if (result != null && result.isValid() && result.getFiducialResults() != null) {
            for (int i = 0; i < result.getFiducialResults().size(); i++) {
                if ((int) result.getFiducialResults().get(i).getFiducialId() == 20) {
                    return result.getFiducialResults().get(i).getTargetYDegrees();
                }
            }
        }
        return 0.0;
    }

    /**
     * Get horizontal offset (tx) for red goal (ID 24)
     */
    public static double getRedGoalX() {
        LLResult result = getLatestResult();
        if (result != null && result.isValid() && result.getFiducialResults() != null) {
            for (int i = 0; i < result.getFiducialResults().size(); i++) {
                if ((int) result.getFiducialResults().get(i).getFiducialId() == 24) {
                    return result.getFiducialResults().get(i).getTargetXDegrees();
                }
            }
        }
        return 0.0;
    }

    /**
     * Get vertical offset (ty) for red goal (ID 24)
     */
    public static double getRedGoalY() {
        LLResult result = getLatestResult();
        if (result != null && result.isValid() && result.getFiducialResults() != null) {
            for (int i = 0; i < result.getFiducialResults().size(); i++) {
                if ((int) result.getFiducialResults().get(i).getFiducialId() == 24) {
                    return result.getFiducialResults().get(i).getTargetYDegrees();
                }
            }
        }
        return 0.0;
    }

    /**
     * Display telemetry for detected AprilTags
     */
    public static void displayTelemetry(OpMode opMode) {
        LLResult result = getLatestResult();
        
        opMode.telemetry.addData("LL Connected", result != null);
        
        if (result != null && result.isValid() && result.getFiducialResults() != null) {
            opMode.telemetry.addData("LL Targets", result.getFiducialResults().size());
            
            for (int i = 0; i < result.getFiducialResults().size(); i++) {
                int tagId = (int) result.getFiducialResults().get(i).getFiducialId();
                double tx = result.getFiducialResults().get(i).getTargetXDegrees();
                
                String goalName = "";
                if (tagId == 20) goalName = " (BLUE GOAL)";
                else if (tagId == 24) goalName = " (RED GOAL)";
                
                opMode.telemetry.addData("Tag " + tagId + goalName, String.format("X: %.2f°", tx));
            }
        } else {
            opMode.telemetry.addData("LL Status", "No targets");
        }
    }
}
