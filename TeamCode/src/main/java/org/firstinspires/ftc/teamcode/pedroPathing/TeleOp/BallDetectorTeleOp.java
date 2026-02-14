package org.firstinspires.ftc.teamcode.pedroPathing.TeleOp;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.pedroPathing.Pipelines.BallDetectorPipeline;
import org.firstinspires.ftc.teamcode.pedroPathing.Pipelines.BallDetectorPipeline.BallDetection;

import java.util.List;

/**
 * TeleOp for live ball detection using the YOLOv8n TFLite model.
 *
 * Displays on the Driver Hub:
 *   - FPS (frames processed per second)
 *   - Total balls detected and color breakdown
 *   - Ball sequence from bottom to top of ramp
 *   - Per-ball details (color, confidence, position)
 *   - First ball ready for intake
 *
 * Controls:
 *   Gamepad1 A  — Toggle live camera view on/off
 *   Gamepad1 B  — Cycle confidence threshold (50% → 70% → 90% → 50%)
 */
@TeleOp(name = "Ball Detector TeleOp", group = "Vision")
public class BallDetectorTeleOp extends LinearOpMode {

    // FPS tracking
    private ElapsedTime fpsTimer = new ElapsedTime();
    private int frameCount = 0;
    private double fps = 0.0;

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
        fpsTimer.reset();

        // --- MAIN LOOP ---
        while (opModeIsActive()) {

            // Handle controls
            handleControls();

            // Poll the model for new detections
            BallDetectorPipeline.pollOnce();

            // Track FPS
            frameCount++;
            double elapsed = fpsTimer.seconds();
            if (elapsed >= 1.0) {
                fps = frameCount / elapsed;
                frameCount = 0;
                fpsTimer.reset();
            }

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
        telemetry.addData("FPS", String.format("%.1f", fps));
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
