package org.firstinspires.ftc.teamcode.pedroPathing.TeleOp.Test_codes;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.pedroPathing.Pipelines.BallDetectorPipeline;
import org.firstinspires.ftc.teamcode.pedroPathing.Pipelines.BallDetectorPipeline.BallDetection;

import java.util.List;

/**
 * TeleOp for live ball detection using the YOLOv8n TFLite model.
 *
 * ════════════════════════════════════════════════════════════════
 *  HOW TO USE BALL DETECTION IN YOUR OWN TELEOP / AUTO
 * ════════════════════════════════════════════════════════════════
 *
 *  1. INIT — call once during setup:
 *       BallDetectorPipeline.init(this, "logitech");
 *     Camera name must match your hardwareMap config.
 *
 *  2. WAIT FOR CAMERA (optional but recommended):
 *       while (!BallDetectorPipeline.isReady()) { sleep(50); }
 *
 *  3. READ DETECTIONS — call in your loop:
 *       List<BallDetection> balls = BallDetectorPipeline.getDetectedBalls();
 *     Balls are sorted bottom-to-top (ramp order).
 *
 *  4. USEFUL GETTERS:
 *       BallDetectorPipeline.getBallCount()        → int
 *       BallDetectorPipeline.getBallSequence()      → "GGP", "PG", etc.
 *       BallDetectorPipeline.getFirstBall()         → BallDetection or null
 *       BallDetectorPipeline.getGreenBalls()        → List<BallDetection>
 *       BallDetectorPipeline.getPurpleBalls()       → List<BallDetection>
 *       BallDetectorPipeline.getComputeMs()         → ms per frame (double)
 *
 *  5. EACH BallDetection HAS:
 *       .label        → "green_ball" or "purple_ball"
 *       .isGreen      → true / false
 *       .confidence   → 0.0 – 1.0
 *       .centerX, .centerY, .width, .height  → pixel coords (640×480)
 *       .bottomY      → bottom edge Y (used for ramp sorting)
 *       .rampPosition → 1 = bottom of ramp, 2, 3 ...
 *
 *  6. SETTINGS:
 *       BallDetectorPipeline.setMinConfidence(0.7f);
 *       BallDetectorPipeline.setLiveViewEnabled(false);
 *
 *  7. CLEANUP — call when done:
 *       BallDetectorPipeline.stop();
 *
 *  REQUIREMENTS:
 *    - Model file: TeamCode/src/main/assets/ftc_ball_nano_320.tflite
 *    - TFLite dep in TeamCode/build.gradle:
 *        implementation 'org.tensorflow:tensorflow-lite:2.4.0'
 * ════════════════════════════════════════════════════════════════
 *
 * Displays on the Driver Hub:
 *   - Compute time in ms per detection cycle
 *   - Colored circles for each detected ball (ramp order)
 *   - Confidence % under each circle
 *
 * Controls:
 *   Gamepad1 A  — Toggle live camera view on/off
 *   Gamepad1 B  — Cycle confidence threshold (50% → 70% → 90% → 50%)
 */
@TeleOp(name = "Ball Detector Test", group = "Test")
public class BallDetectorTest extends LinearOpMode {

    // Confidence cycling
    private final float[] CONFIDENCE_LEVELS = {0.50f, 0.70f, 0.90f};
    private int confidenceIndex = 0;

    // Toggle states
    private boolean liveViewEnabled = true;
    private boolean prevA = false;
    private boolean prevB = false;

    @Override
    public void runOpMode() {

        // --- INIT ---
        telemetry.setDisplayFormat(Telemetry.DisplayFormat.HTML);
        telemetry.addData("Status", "Initializing Ball Detector...");
        telemetry.update();

        BallDetectorPipeline.init(this, "logitech");

        // Wait for camera to come online
        while (!isStarted() && !isStopRequested()) {
            if (BallDetectorPipeline.isReady()) {
                telemetry.addData("Status", "✅ Camera Ready");
            } else {
                telemetry.addData("Status", "⏳ Waiting for camera...");
            }

            BallDetectorPipeline.pollOnce();
            showTelemetry();

            telemetry.addData("", "——————————————");
            telemetry.addData(">", "Press ▶ START to begin");
            telemetry.update();
            sleep(50);
        }

        waitForStart();

        // --- MAIN LOOP ---
        while (opModeIsActive()) {

            // Handle controls
            handleControls();

            // Poll the model for new detections
            BallDetectorPipeline.pollOnce();

            // Display everything on the Driver Hub
            showTelemetry();
            telemetry.update();
        }

        // --- CLEANUP ---
        BallDetectorPipeline.stop();
    }

    // HTML colored circle for a ball
    private String ballCircle(boolean isGreen) {
        String color = isGreen ? "#00FF00" : "#B000FF";
        return String.format(
            "<span style='color:%s; font-size:48px;'>&#x2B24;</span>", color);
    }

    // HTML colored text
    private String colored(String text, String hex) {
        return String.format("<span style='color:%s;'>%s</span>", hex, text);
    }

    /**
     * Renders colored circles in ramp order with confidence %.
     */
    private void showTelemetry() {

        List<BallDetection> balls = BallDetectorPipeline.getDetectedBalls();

        // ── Colored circles in a row ──
        if (balls.isEmpty()) {
            telemetry.addData("", "<i>no balls detected</i>");
        } else {
            StringBuilder visual = new StringBuilder();
            for (int i = 0; i < balls.size(); i++) {
                visual.append(ballCircle(balls.get(i).isGreen));
                if (i < balls.size() - 1) visual.append("  ");
            }
            telemetry.addData("", visual.toString());

            // Confidence % under each circle
            StringBuilder confs = new StringBuilder();
            for (int i = 0; i < balls.size(); i++) {
                BallDetection b = balls.get(i);
                String c = b.isGreen ? "#00FF00" : "#B000FF";
                confs.append(colored(String.format("%.0f%%", b.confidence * 100), c));
                if (i < balls.size() - 1) confs.append("&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;");
            }
            telemetry.addData("", confs.toString());
        }

        telemetry.addData("", "");
        telemetry.addData("Compute", String.format("%.0f ms", BallDetectorPipeline.getComputeMs()));
    }

    /**
     * Reads gamepad1 buttons for toggling settings.
     */
    private void handleControls() {
        // A — toggle live camera view
        boolean currA = gamepad1.a;
        if (currA && !prevA) {
            liveViewEnabled = !liveViewEnabled;
            BallDetectorPipeline.setLiveViewEnabled(liveViewEnabled);
        }
        prevA = currA;

        // B — cycle confidence threshold
        boolean currB = gamepad1.b;
        if (currB && !prevB) {
            confidenceIndex = (confidenceIndex + 1) % CONFIDENCE_LEVELS.length;
            BallDetectorPipeline.setMinConfidence(CONFIDENCE_LEVELS[confidenceIndex]);
        }
        prevB = currB;
    }
}
