package org.firstinspires.ftc.teamcode.pedroPathing.Pipelines;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.LLStatus;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;

import java.util.List;

/**
 * Limelight Pipeline
 * 
 * This pipeline initializes and manages a Limelight 3A camera
 * configured for AprilTag detection.
 */
public class Limelight_Pipeline {

    // Limelight camera
    public static Limelight3A limelight;
    
    private OpMode opMode;

    /**
     * Constructor - initializes the Limelight 3A and starts it automatically
     * @param opMode The OpMode using this pipeline
     */
    public Limelight_Pipeline(OpMode opMode) {
        this.opMode = opMode;
        
        // Initialize and start Limelight
        initLimelight(opMode);
    }

    /**
     * Get the latest result from the Limelight
     * @return LLResult containing detection data
     */
    public LLResult getLatestResult() {
        return limelight.getLatestResult();
    }

    /**
     * Get the Limelight status
     * @return LLStatus containing camera status
     */
    public LLStatus getStatus() {
        return limelight.getStatus();
    }

    /**
     * Get AprilTag (fiducial) detections from latest result
     * @return List of detected AprilTags, or null if no valid data
     */
    public List<LLResultTypes.FiducialResult> getAprilTags() {
        LLResult result = limelight.getLatestResult();
        if (result != null && result.isValid()) {
            return result.getFiducialResults();
        }
        return null;
    }

    /**
     * Check if any AprilTags are detected
     * @return true if at least one AprilTag is visible
     */
    public boolean hasAprilTags() {
        List<LLResultTypes.FiducialResult> tags = getAprilTags();
        return tags != null && !tags.isEmpty();
    }

    /**
     * Get a specific AprilTag by ID
     * @param tagId The AprilTag ID to find
     * @return The FiducialResult for the tag, or null if not found
     */
    public LLResultTypes.FiducialResult getAprilTagById(int tagId) {
        List<LLResultTypes.FiducialResult> tags = getAprilTags();
        if (tags != null) {
            for (LLResultTypes.FiducialResult tag : tags) {
                if (tag.getFiducialId() == tagId) {
                    return tag;
                }
            }
        }
        return null;
    }

    /**
     * Switch to a different Limelight pipeline
     * @param pipelineIndex Pipeline number (0-9)
     */
    public void switchPipeline(int pipelineIndex) {
        limelight.pipelineSwitch(pipelineIndex);
    }

    /**
     * Update telemetry with Limelight status and AprilTag detections
     */
    public void updateTelemetry() {
        LLStatus status = limelight.getStatus();
        opMode.telemetry.addData("Limelight", status.getName());
        opMode.telemetry.addData("LL Temp", "%.1f°C", status.getTemp());
        opMode.telemetry.addData("LL CPU", "%.1f%%", status.getCpu());
        opMode.telemetry.addData("LL FPS", "%d", (int)status.getFps());
        opMode.telemetry.addData("Pipeline", "%d (%s)", status.getPipelineIndex(), status.getPipelineType());

        LLResult result = limelight.getLatestResult();
        if (result != null && result.isValid()) {
            List<LLResultTypes.FiducialResult> tags = result.getFiducialResults();
            
            if (tags != null && !tags.isEmpty()) {
                opMode.telemetry.addData("AprilTags Detected", tags.size());
                
                for (LLResultTypes.FiducialResult tag : tags) {
                    opMode.telemetry.addData("Tag " + tag.getFiducialId(), 
                        "X: %.1f°, Y: %.1f°", 
                        tag.getTargetXDegrees(), 
                        tag.getTargetYDegrees());
                }
            } else {
                opMode.telemetry.addData("AprilTags", "None detected");
            }
            
            opMode.telemetry.addData("Latency", "%.1f ms", 
                result.getCaptureLatency() + result.getTargetingLatency());
        } else {
            opMode.telemetry.addData("Limelight", "No valid data");
        }
    }

    /**
     * Stop the Limelight (call in OpMode stop())
     */
    public void stop() {
        if (limelight != null) {
            limelight.stop();
        }
    }

    /**
     * Static method to initialize Limelight
     * @param opMode The OpMode
     */
    public static void initLimelight(OpMode opMode) {
        limelight = opMode.hardwareMap.get(Limelight3A.class, "limelight");
        limelight.pipelineSwitch(0);
        limelight.start();
    }
}
