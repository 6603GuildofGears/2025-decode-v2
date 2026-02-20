package org.firstinspires.ftc.teamcode.pedroPathing.TeleOp.Test_codes.Turret;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.TouchSensor;

import org.firstinspires.ftc.teamcode.pedroPathing.Pipelines.TurretSubsystem;

/**
 * ========================================================================
 *  TURRET MANUAL TEST — Simple verification OpMode
 * ========================================================================
 *
 * Use this AFTER running the auto-tuner to verify that the tuned PID gains
 * produce smooth, accurate, repeatable turret movement.
 *
 * ========================== CONTROLS ==========================
 *
 *   D-pad LEFT   →  Command -30° (relative to current target)
 *   D-pad RIGHT  →  Command +30° (relative to current target)
 *   D-pad UP     →  Command +10° (fine adjustment)
 *   D-pad DOWN   →  Command -10° (fine adjustment)
 *   A button     →  Go to center (165°)
 *   B button     →  Go to 0° (home position)
 *   X button     →  Re-home (run homing routine again)
 *   Y button     →  Go to 330° (max position)
 *
 * ========================== TESTING PROCEDURE ==========================
 *
 * Step 1: Run TurretPIDTuner OpMode, let it auto-tune (5-10 iterations).
 * Step 2: Note final P, I, D values from telemetry.
 * Step 3: Enter those values below in TUNED_P, TUNED_I, TUNED_D.
 * Step 4: Run this OpMode (TurretManualTest).
 * Step 5: Manually command to various angles and verify:
 *         - Smooth motion (no jerking)
 *         - Minimal overshoot (<5°)
 *         - Fast settling (<1 second)
 *         - Accurate hold (±2°)
 * Step 6: Test soft limits — try to go below 0° and above 330°.
 *         The turret should refuse and stay within range.
 * Step 7: Test homing reliability — press X five times in a row.
 *         The turret must home successfully every time.
 * Step 8: Walk around the turret while it holds position.
 *         It should NOT move — this is robot-relative mode.
 *
 * ========================== GAINS ==========================
 *
 * Enter your tuned PID gains here after running the auto-tuner.
 * These are the STARTING DEFAULTS — replace with your tuned values!
 */
@Disabled
@TeleOp(name = "Turret Manual Test", group = "Test")
public class TurretManualTest extends LinearOpMode {

    // =====================================================
    // ENTER YOUR TUNED PID GAINS HERE
    // (from TurretPIDTuner telemetry output)
    // =====================================================
    private static final double TUNED_P = 0.015;
    private static final double TUNED_I = 0.0003;
    private static final double TUNED_D = 0.003;

    // Max motor power — can increase after verifying stable behavior
    private static final double MAX_POWER = 0.7;

    /** Step size for D-pad coarse control (degrees). */
    private static final double COARSE_STEP = 30.0;

    /** Step size for D-pad fine control (degrees). */
    private static final double FINE_STEP = 10.0;

    // ========================== MAIN ==========================

    @Override
    public void runOpMode() throws InterruptedException {

        // --- Hardware ---
        DcMotorEx turretMotor = hardwareMap.get(DcMotorEx.class, "turret");
        turretMotor.setDirection(DcMotor.Direction.FORWARD);

        TouchSensor mag = hardwareMap.get(TouchSensor.class, "mag");

        TurretSubsystem turret = new TurretSubsystem(turretMotor, mag);
        turret.setPIDGains(TUNED_P, TUNED_I, TUNED_D);
        turret.setMaxPower(MAX_POWER);

        // --- Home ---
        telemetry.addLine("=== TURRET MANUAL TEST ===");
        telemetry.addLine("Homing...");
        telemetry.update();

        boolean homeOk = turret.home();

        telemetry.addLine(homeOk ? "Homing OK!" : "!! HOMING FAILED !!");
        telemetry.addLine("Press PLAY to start.");
        telemetry.update();

        waitForStart();
        if (isStopRequested()) return;

        // Start at center
        double targetAngle = TurretSubsystem.CENTER_ANGLE_DEG;
        turret.setTargetAngle(targetAngle);

        // Button edge detection — prevent repeated triggers from holding button
        boolean lastDpadLeft = false;
        boolean lastDpadRight = false;
        boolean lastDpadUp = false;
        boolean lastDpadDown = false;
        boolean lastA = false;
        boolean lastB = false;
        boolean lastX = false;
        boolean lastY = false;

        // === Main loop ===
        while (opModeIsActive()) {

            // --- Read buttons (edge-triggered) ---
            boolean dpadLeftPressed  = gamepad1.dpad_left  && !lastDpadLeft;
            boolean dpadRightPressed = gamepad1.dpad_right && !lastDpadRight;
            boolean dpadUpPressed    = gamepad1.dpad_up    && !lastDpadUp;
            boolean dpadDownPressed  = gamepad1.dpad_down  && !lastDpadDown;
            boolean aPressed = gamepad1.a && !lastA;
            boolean bPressed = gamepad1.b && !lastB;
            boolean xPressed = gamepad1.x && !lastX;
            boolean yPressed = gamepad1.y && !lastY;

            lastDpadLeft  = gamepad1.dpad_left;
            lastDpadRight = gamepad1.dpad_right;
            lastDpadUp    = gamepad1.dpad_up;
            lastDpadDown  = gamepad1.dpad_down;
            lastA = gamepad1.a;
            lastB = gamepad1.b;
            lastX = gamepad1.x;
            lastY = gamepad1.y;

            // --- Process commands ---

            // Coarse: ±30°
            if (dpadLeftPressed)  targetAngle -= COARSE_STEP;
            if (dpadRightPressed) targetAngle += COARSE_STEP;

            // Fine: ±10°
            if (dpadDownPressed) targetAngle -= FINE_STEP;
            if (dpadUpPressed)   targetAngle += FINE_STEP;

            // Preset positions
            if (aPressed) targetAngle = TurretSubsystem.CENTER_ANGLE_DEG; // 165°
            if (bPressed) targetAngle = 0;                                 // Home
            if (yPressed) targetAngle = TurretSubsystem.MAX_ANGLE_DEG;    // 330°

            // Re-home
            if (xPressed) {
                telemetry.clearAll();
                telemetry.addLine("Re-homing...");
                telemetry.update();
                turret.home();
                targetAngle = TurretSubsystem.CENTER_ANGLE_DEG;
                turret.setTargetAngle(targetAngle);
                continue;
            }

            // Clamp target to valid range
            targetAngle = Math.max(TurretSubsystem.MIN_ANGLE_DEG,
                          Math.min(targetAngle, TurretSubsystem.MAX_ANGLE_DEG));

            // Command turret and update PID
            turret.setTargetAngle(targetAngle);
            turret.update();

            // --- Telemetry ---
            telemetry.clearAll();
            telemetry.addLine("=== TURRET MANUAL TEST ===");
            telemetry.addLine("");

            telemetry.addData("Target", "%.1f°", targetAngle);
            telemetry.addData("Current", "%.1f°", turret.getCurrentAngle());
            telemetry.addData("Error", "%.1f°", turret.getError());
            telemetry.addData("Motor Power", "%.3f", turret.getMotorPower());
            telemetry.addData("At Target (±2°)", turret.isAtTarget());
            telemetry.addLine("");

            telemetry.addData("Encoder", "%d ticks", turret.getEncoderTicks());
            telemetry.addData("Homed", turret.isHomed());
            telemetry.addData("Stalled", turret.isStalled());
            telemetry.addLine("");

            telemetry.addData("PID Gains", "P=%.5f  I=%.6f  D=%.5f",
                turret.getKP(), turret.getKI(), turret.getKD());
            telemetry.addLine("");

            telemetry.addLine("--- Controls ---");
            telemetry.addLine("D-pad L/R: ±30°  |  D-pad U/D: ±10°");
            telemetry.addLine("[A] Center (165°)  |  [B] Home (0°)");
            telemetry.addLine("[Y] Max (330°)     |  [X] Re-home");

            // Stall warning
            if (turret.isStalled()) {
                telemetry.addLine("");
                telemetry.addLine("!! STALL DETECTED — motor stopped !!");
                telemetry.addLine("Press X to re-home and clear.");
                turret.clearStall();
            }

            telemetry.update();
            sleep(20);
        }

        // Cleanup
        turret.emergencyStop();
    }
}
