package org.firstinspires.ftc.teamcode.pedroPathing.Pipelines;

import android.graphics.Bitmap;
import android.util.Size;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.tfod.TfodProcessor;

import java.util.ArrayList;
import java.util.Collections;
import java.util.Comparator;
import java.util.List;

/**
 * Ball Detection Pipeline for FTC DECODE
 * 
 * Uses a custom YOLOv8n TFLite model to detect green and purple balls on the ramp.
 * Returns balls sorted by Y position (lowest/bottom first) for ramp order.
 * 
 * Usage:
 *   BallDetectorPipeline.init(opMode);
 *   // In loop:
 *   List<BallDetection> balls = BallDetectorPipeline.getDetectedBalls();
 *   // balls are sorted bottom-to-top (first ball is at bottom of ramp)
 */
public class BallDetectorPipeline {

    // Model configuration
    private static final String MODEL_ASSET = "ftc_ball_nano_320.tflite";
    private static final String[] LABELS = {"green_ball", "purple_ball"};
    private static final int MODEL_INPUT_SIZE = 320;
    
    // Detection thresholds
    private static final float MIN_CONFIDENCE = 0.5f;
    
    // Vision components
    private static TfodProcessor tfod;
    private static VisionPortal visionPortal;
    
    // Cached detections
    private static List<BallDetection> cachedDetections = new ArrayList<>();
    private static boolean cacheValid = false;

    /**
     * Represents a detected ball with its properties
     */
    public static class BallDetection {
        public final String label;           // "green_ball" or "purple_ball"
        public final boolean isGreen;        // true if green, false if purple
        public final float confidence;       // 0.0 to 1.0
        public final float centerX;          // Center X in pixels
        public final float centerY;          // Center Y in pixels
        public final float width;            // Bounding box width
        public final float height;           // Bounding box height
        public final float bottomY;          // Bottom edge Y (for sorting by ramp position)
        public final int rampPosition;       // 1 = bottom of ramp, 2 = next, etc.

        public BallDetection(Recognition recognition, int position) {
            this.label = recognition.getLabel();
            this.isGreen = label.equals("green_ball");
            this.confidence = recognition.getConfidence();
            this.centerX = (recognition.getLeft() + recognition.getRight()) / 2f;
            this.centerY = (recognition.getTop() + recognition.getBottom()) / 2f;
            this.width = recognition.getWidth();
            this.height = recognition.getHeight();
            this.bottomY = recognition.getBottom();  // Higher Y = lower on screen = lower on ramp
            this.rampPosition = position;
        }

        @Override
        public String toString() {
            return String.format("%s #%d (%.0f%%) at (%.0f, %.0f)", 
                isGreen ? "GREEN" : "PURPLE", 
                rampPosition, 
                confidence * 100, 
                centerX, centerY);
        }
    }

    /**
     * Initialize the ball detection pipeline with a Logitech C920 webcam.
     * 
     * @param opMode The OpMode to get hardware from
     */
    public static void init(OpMode opMode) {
        init(opMode, "Webcam 1");
    }

    /**
     * Initialize the ball detection pipeline.
     * 
     * @param opMode The OpMode to get hardware from
     * @param webcamName The name of the webcam in the hardware config
     */
    public static void init(OpMode opMode, String webcamName) {
        // Create the TensorFlow Object Detection processor
        tfod = new TfodProcessor.Builder()
                .setModelFileName(MODEL_ASSET)
                .setModelLabels(LABELS)
                .setModelInputSize(MODEL_INPUT_SIZE)
                .setModelAspectRatio(1.0)  // Square input (320x320)
                .build();

        // Set confidence threshold
        tfod.setMinResultConfidence(MIN_CONFIDENCE);

        // Create the vision portal
        visionPortal = new VisionPortal.Builder()
                .setCamera(opMode.hardwareMap.get(WebcamName.class, webcamName))
                .setCameraResolution(new Size(640, 480))
                .addProcessor(tfod)
                .setStreamFormat(VisionPortal.StreamFormat.MJPEG)  // Lower bandwidth
                .enableLiveView(true)
                .setAutoStopLiveView(true)
                .build();

        opMode.telemetry.addData("Ball Detector", "Initialized");
        opMode.telemetry.addData("Model", MODEL_ASSET);
        opMode.telemetry.update();
    }

    /**
     * Poll for new detections. Call this once per loop iteration.
     */
    public static void pollOnce() {
        if (tfod == null) {
            cacheValid = false;
            return;
        }

        List<Recognition> recognitions = tfod.getRecognitions();
        cachedDetections.clear();

        if (recognitions != null && !recognitions.isEmpty()) {
            // Sort by bottom Y coordinate (highest Y value = bottom of screen = bottom of ramp)
            // This gives us the order from bottom of ramp to top
            List<Recognition> sorted = new ArrayList<>(recognitions);
            Collections.sort(sorted, new Comparator<Recognition>() {
                @Override
                public int compare(Recognition a, Recognition b) {
                    // Higher bottom Y = lower on ramp = comes first
                    return Float.compare(b.getBottom(), a.getBottom());
                }
            });

            // Create BallDetection objects with position numbers
            int position = 1;
            for (Recognition rec : sorted) {
                cachedDetections.add(new BallDetection(rec, position));
                position++;
            }
        }
        cacheValid = true;
    }

    /**
     * Get all detected balls, sorted by ramp position (bottom first).
     * Call pollOnce() before this each loop iteration.
     * 
     * @return List of detected balls, sorted by Y position (bottom of ramp first)
     */
    public static List<BallDetection> getDetectedBalls() {
        if (!cacheValid) pollOnce();
        return new ArrayList<>(cachedDetections);
    }

    /**
     * Get only green balls, sorted by ramp position.
     */
    public static List<BallDetection> getGreenBalls() {
        List<BallDetection> greens = new ArrayList<>();
        for (BallDetection ball : getDetectedBalls()) {
            if (ball.isGreen) greens.add(ball);
        }
        return greens;
    }

    /**
     * Get only purple balls, sorted by ramp position.
     */
    public static List<BallDetection> getPurpleBalls() {
        List<BallDetection> purples = new ArrayList<>();
        for (BallDetection ball : getDetectedBalls()) {
            if (!ball.isGreen) purples.add(ball);
        }
        return purples;
    }

    /**
     * Get the color sequence of balls from bottom to top of ramp.
     * 
     * @return String like "GPGP" (G=green, P=purple) from bottom to top
     */
    public static String getBallSequence() {
        StringBuilder sb = new StringBuilder();
        for (BallDetection ball : getDetectedBalls()) {
            sb.append(ball.isGreen ? "G" : "P");
        }
        return sb.toString();
    }

    /**
     * Get the first (bottom-most) ball on the ramp.
     * 
     * @return The bottom ball, or null if no balls detected
     */
    public static BallDetection getFirstBall() {
        List<BallDetection> balls = getDetectedBalls();
        return balls.isEmpty() ? null : balls.get(0);
    }

    /**
     * Check if there are any balls detected.
     */
    public static boolean hasBalls() {
        return !getDetectedBalls().isEmpty();
    }

    /**
     * Get the total number of detected balls.
     */
    public static int getBallCount() {
        return getDetectedBalls().size();
    }

    /**
     * Set the minimum confidence threshold for detections.
     * 
     * @param confidence Value between 0.0 and 1.0 (default is 0.5)
     */
    public static void setMinConfidence(float confidence) {
        if (tfod != null) {
            tfod.setMinResultConfidence(confidence);
        }
    }

    /**
     * Enable or disable the live camera stream.
     */
    public static void setLiveViewEnabled(boolean enabled) {
        if (visionPortal != null) {
            if (enabled) {
                visionPortal.resumeLiveView();
            } else {
                visionPortal.stopLiveView();
            }
        }
    }

    /**
     * Stop the vision portal and release resources.
     */
    public static void stop() {
        if (visionPortal != null) {
            visionPortal.close();
            visionPortal = null;
        }
        tfod = null;
        cachedDetections.clear();
        cacheValid = false;
    }

    /**
     * Check if the pipeline is initialized and ready.
     */
    public static boolean isReady() {
        return visionPortal != null && 
               visionPortal.getCameraState() == VisionPortal.CameraState.STREAMING;
    }
}
