package org.firstinspires.ftc.teamcode.pedroPathing.Pipelines;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;

public class Limelight_Pipeline {

    public static Limelight3A limelight;
    private static boolean isInitialized = false;

    /**
     * Initialize the Limelight during the init period
     * Sets it to AprilTag mode (pipeline 0) and turns off LEDs to save power
     * If already initialized from a previous OpMode, reconfigures it
     */
    public static void initLimelight(OpMode opMode) {
        if (!isInitialized) {
            // Get the Limelight from hardware map
            limelight = opMode.hardwareMap.get(Limelight3A.class, "limelight");
            limelight.start();
            isInitialized = true;
        }
        
        // Always configure settings (even if already initialized from auto)
        // Set to AprilTag pipeline (typically pipeline 0)
        limelight.pipelineSwitch(0);

        // Turn off LEDs to save power
        limelight.stop(); // Stop updates temporarily
        limelight.start(); // Restart with new settings

        opMode.telemetry.addData("Limelight", "Configured - AprilTag Mode");
        opMode.telemetry.addData("LED Status", "OFF (power saving)");
        opMode.telemetry.update();
    }

    /**
     * Get the latest result from the Limelight
     * Call this during TeleOp to get AprilTag detection data
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
     * Display telemetry showing detected AprilTag ID and pattern
     * Call this in your TeleOp loop to see what the Limelight detects
     */
    public static void displayTelemetry(OpMode opMode) {
        LLResult result = getLatestResult();
        
        if (result != null && result.isValid()) {
            // Check if we have any fiducial (AprilTag) detections
            if (result.getFiducialResults() != null && !result.getFiducialResults().isEmpty()) {
                // Get the first detected tag
                int tagId = (int) result.getFiducialResults().get(0).getFiducialId();
                String pattern = getColorPattern(tagId);
                
                opMode.telemetry.addData("Limelight", "Target Detected");
                
                // Display differently for motif vs goal tags
                if (tagId >= 21 && tagId <= 23) {
                    // Motif tags - show the color order prominently
                    opMode.telemetry.addData("Motif Pattern", pattern);
                    opMode.telemetry.addData("Obelisk ID", tagId);
                } else if (tagId == 20 || tagId == 24) {
                    // Goal tags
                    opMode.telemetry.addData("Goal", pattern);
                    opMode.telemetry.addData("Tag ID", tagId);
                } else {
                    // Other tags
                    opMode.telemetry.addData("AprilTag ID", tagId);
                }
            } else {
                opMode.telemetry.addData("Limelight", "No AprilTag Detected");
            }
        } else {
            opMode.telemetry.addData("Limelight", "No Target");
        }
    }

    /**
     * Stop the Limelight (optional - call at end of OpMode if needed)
     */
    public static void stopLimelight() {
        if (limelight != null) {
            limelight.stop();
            isInitialized = false;
        }
    }

    /**
     * Reset the initialization flag (useful for switching between OpModes)
     */
    public static void reset() {
        isInitialized = false;
    }

    /**
     * Get the specimen motif color pattern for a given AprilTag ID
     * Returns a string describing the color order: Green (G) and Purple (P)
     * NOTE: This is ONLY the motif pattern, not scoring instructions
     * 
     * @param tagId The AprilTag ID detected
     * @return String describing motif color pattern, or "Unknown" if ID not recognized
     */
    public static String getColorPattern(int tagId) {
        switch (tagId) {
            case 20:
                return "Blue Goal"; // Blue alliance goal
            case 21:
                return "Green-Purple-Purple"; // Motif only
            case 22:
                return "Purple-Green-Purple"; // Motif only
            case 23:
                return "Purple-Purple-Green"; // Motif only
            case 24:
                return "Red Goal"; // Red alliance goal
            default:
                return "Unknown ID";
        }
    }

    /**
     * Get specimen motif color pattern as array for easier programmatic access
     * NOTE: This is ONLY the motif pattern, not scoring instructions
     * @param tagId The AprilTag ID detected
     * @return Array of color strings ["Green", "Purple", etc.] or empty array if unknown
     */
    public static String[] getColorPatternArray(int tagId) {
        switch (tagId) {
            case 20:
                return new String[]{"Blue", "Goal"}; // Blue alliance goal
            case 21:
                return new String[]{"Green", "Purple", "Purple"}; // Motif only
            case 22:
                return new String[]{"Purple", "Green", "Purple"}; // Motif only
            case 23:
                return new String[]{"Purple", "Purple", "Green"}; // Motif only
            case 24:
                return new String[]{"Red", "Goal"}; // Red alliance goal
            default:
                return new String[]{};
        }
    }
}
