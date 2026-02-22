package org.firstinspires.ftc.teamcode.pedroPathing.TeleOp;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.pedroPathing.TeleOp.Test_codes.BallDetectorPipeline;
import org.firstinspires.ftc.teamcode.pedroPathing.TeleOp.Test_codes.BallDetectorPipeline.BallDetection;
import org.firstinspires.ftc.robotcore.external.Telemetry;

import java.util.List;

/**
 * TeleOp with mecanum drive AND live ball detection using the Logitech camera
 * and the YOLOv8/v11 ONNX model (ftc_ball_v3).
 *
 * Drive Controls (Gamepad 1):
 *   Left Stick   — Strafe (X/Y translation)
 *   Right Stick X — Rotation
 *   D-Pad        — Slow cardinal movement
 *
 * Vision Controls (Gamepad 1):
 *   A  — Toggle live camera view on/off
 *   B  — Cycle confidence threshold (50% → 70% → 90% → 50%)
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

    // Drive speed modifier
    private double gear = 1.25;

    // Drive motors (only the 4 wheel motors)
    private DcMotorEx frontLeft;
    private DcMotorEx frontRight;
    private DcMotorEx backLeft;
    private DcMotorEx backRight;

    @Override
    public void runOpMode() {

        // --- INIT ---
        telemetry.setDisplayFormat(Telemetry.DisplayFormat.HTML);
        telemetry.addData("Status", "Initializing...");
        telemetry.update();

        // Initialize only the 4 drive motors
        frontLeft  = hardwareMap.get(DcMotorEx.class, "frontLeft");
        frontRight = hardwareMap.get(DcMotorEx.class, "frontRight");
        backRight  = hardwareMap.get(DcMotorEx.class, "backRight");
        backLeft   = hardwareMap.get(DcMotorEx.class, "backLeft");

        frontLeft.setDirection(DcMotor.Direction.FORWARD);
        frontRight.setDirection(DcMotor.Direction.REVERSE);
        backRight.setDirection(DcMotor.Direction.REVERSE);
        backLeft.setDirection(DcMotor.Direction.REVERSE);

        frontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // Initialize ball detector camera (Logitech webcam) with ONNX model
        BallDetectorPipeline.init(this, "logitech");

        // Wait for camera to come online
        while (!isStarted() && !isStopRequested()) {
            if (BallDetectorPipeline.isReady()) {
                telemetry.addData("Status", "✅ Camera & Motors Ready");
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

            // ── DRIVE ──
            handleDrive();

            // ── VISION CONTROLS ──
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

    /**
     * Mecanum drive using gamepad1, matching Cassius_Blue controls.
     */
    private void handleDrive() {
        double LStickY = gamepad1.left_stick_y;
        double LStickX = -gamepad1.left_stick_x;
        double RStickX = -gamepad1.right_stick_x;

        boolean dpadUp    = gamepad1.dpad_up;
        boolean dpadDown  = gamepad1.dpad_down;
        boolean dpadLeft  = gamepad1.dpad_left;
        boolean dpadRight = gamepad1.dpad_right;

        if (Math.abs(LStickX) > 0 || Math.abs(LStickY) > 0 || Math.abs(RStickX) > 0) {
            // Mecanum math
            double r = Math.hypot(LStickX, LStickY);
            double robotAngle = Math.atan2(LStickY, LStickX) - Math.PI / 4;
            double rightX = RStickX;

            double v1 = r * Math.cos(robotAngle) + rightX * gear; // LF
            double v2 = r * Math.sin(robotAngle) - rightX * gear; // RF
            double v3 = r * Math.sin(robotAngle) + rightX * gear; // LB
            double v4 = r * Math.cos(robotAngle) - rightX * gear; // RB

            setDrivePower(v1, v3, v2, v4);

        } else if (dpadUp) {
            setDrivePower(1, 1, 1, 1);
        } else if (dpadRight) {
            setDrivePower(1, -1, -1, 1);
        } else if (dpadLeft) {
            setDrivePower(-1, 1, 1, -1);
        } else if (dpadDown) {
            setDrivePower(-1, -1, -1, -1);
        } else {
            setDrivePower(0, 0, 0, 0);
        }
    }

    /**
     * Sets power to the 4 drive motors.
     */
    private void setDrivePower(double lf, double lb, double rf, double rb) {
        frontLeft.setPower(lf);
        backLeft.setPower(lb);
        frontRight.setPower(rf);
        backRight.setPower(rb);
    }

    /**
     * Renders minimal telemetry with colored dots for detected balls.
     */
    private void showTelemetry() {

        List<BallDetection> balls = BallDetectorPipeline.getDetectedBalls();


        telemetry.addData("Inference", String.format("%.1f ms", BallDetectorPipeline.getComputeMs()));
        telemetry.addData("Balls", balls.size());

        // Colored dots row
        if (balls.isEmpty()) {
            telemetry.addData("", "<i>none</i>");
        } else {
            StringBuilder dots = new StringBuilder();
            for (int i = 0; i < balls.size(); i++) {
                String color = balls.get(i).isGreen ? "#00FF00" : "#B000FF";
                dots.append(String.format("<span style='color:%s; font-size:48px;'>&#x2B24;</span>", color));
                if (i < balls.size() - 1) dots.append("  ");
            }
            telemetry.addData("", dots.toString());
        }

        // Sequence
        String seq = BallDetectorPipeline.getBallSequence();
        telemetry.addData("Sequence", seq.isEmpty() ? "—" : seq);
    }

    /**
     * Reads gamepad1 buttons for toggling vision settings.
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
