package org.firstinspires.ftc.teamcode.pedroPathing.TeleOp.Test_codes;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;

import static org.firstinspires.ftc.teamcode.pedroPathing.Pipelines.Motor_PipeLine.*;
import static org.firstinspires.ftc.teamcode.pedroPathing.Pipelines.Limelight_Pipeline.*;
import static org.firstinspires.ftc.teamcode.pedroPathing.Pipelines.Sensor.*;

/**
 * SIMPLE turret tracker — proves the concept works with nothing fancy.
 *
 * How it works:
 *   1. Limelight sees AprilTag → tx tells us how many degrees off-center the target is
 *   2. turretPower = kP * tx  (proportional control, that's it)
 *   3. Safety limits prevent turret from going past physical stops
 *
 * Controls:
 *   Left stick  = drive (mecanum)
 *   Right stick = turn
 *   Turret auto-tracks whenever it sees the blue goal tag (ID 20)
 *
 * TUNING:
 *   kP  — how aggressively the turret chases the target
 *         Too low = sluggish. Too high = oscillates around target.
 *         Start at 0.04, increase until it tracks well without oscillating.
 *
 *   DEADBAND — ignore errors smaller than this (degrees)
 *              1° is reasonable. Lower if you need more precision.
 */
@Configurable
@TeleOp(name = "Simple Turret Tracker", group = "Testing")
public class Simple_Turret_Tracker extends LinearOpMode {

    // ===== TUNING (live via Panels) =====
    public static double kP = 0.008;         // Power per degree of error
    public static double DEADBAND = 1.0;     // Degrees — ignore errors smaller than this
    public static double MAX_POWER = 0.4;    // Maximum turret power (0-1)
    public static double MIN_POWER = 0.02;   // Minimum power to overcome static friction
    public static double K_HL = 0.008;       // Heading lock gain (power per degree of chassis rotation)

    // ===== TURRET LIMITS (don't change unless hardware changes) =====
    static final double TICKS_PER_DEG = 2.64;  // 950.4 ticks / 360°
    static final double MIN_DEG = 0;
    static final double MAX_DEG = 322;
    static final double SLOW_ZONE_DEG = 11.4;  // Ramp down near limits

    @Override
    public void runOpMode() throws InterruptedException {

        // Init hardware
        intMotors(this);
        initLimelight(this);
        initSensors(this);

        // Zero turret encoder at current position
        turret.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        turret.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        // IMU for heading lock
        IMU imu = hardwareMap.get(IMU.class, "imu");
        imu.initialize(new IMU.Parameters(new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.UP,
                RevHubOrientationOnRobot.UsbFacingDirection.FORWARD)));
        imu.resetYaw();

        // Heading lock state
        double lockedHeading = 0;
        boolean headingLocked = false;

        TelemetryManager telemetryM = PanelsTelemetry.INSTANCE.getTelemetry();

        telemetry.addData("Status", "Ready — turret will auto-track blue goal");
        telemetry.addData("kP", kP);
        telemetry.addData("Deadband", DEADBAND + "°");
        telemetry.addData("Panels", "Tune kP, DEADBAND, K_HL live");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {

            // === POLL LIMELIGHT ===
            pollOnce();

            // === DRIVE (simple mecanum) ===
            double lx = -gamepad1.left_stick_x;
            double ly = gamepad1.left_stick_y;
            double rx = -gamepad1.right_stick_x;
            double r = Math.hypot(lx, ly);
            double angle = Math.atan2(ly, lx) - Math.PI / 4;
            double gear = 1.25;
            SetPower(
                r * Math.cos(angle) + rx * gear,
                r * Math.sin(angle) + rx * gear,
                r * Math.sin(angle) - rx * gear,
                r * Math.cos(angle) - rx * gear
            );

            // === TURRET TRACKING ===
            int turretPos = turret.getCurrentPosition();
            double turretDeg = turretPos / TICKS_PER_DEG;
            double turretPower = 0;
            String trackingState = "NO TARGET";
            double heading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);
            double headingError = 0;

            boolean seesGoal = hasBlueGoal();

            if (seesGoal) {
                double tx = getBlueGoalX();  // Degrees off-center (negative = left, positive = right)

                if (Math.abs(tx) > DEADBAND) {
                    // Simple P-control: power proportional to error
                    turretPower = kP * tx;

                    // Ensure minimum power to overcome static friction
                    if (Math.abs(turretPower) < MIN_POWER) {
                        turretPower = Math.signum(turretPower) * MIN_POWER;
                    }

                    // Clamp to max power
                    turretPower = Math.max(-MAX_POWER, Math.min(MAX_POWER, turretPower));

                    trackingState = String.format("TRACKING (tx=%.1f°, pwr=%.2f)", tx, turretPower);
                } else {
                    // On target — lock the heading so we hold aim during spin
                    lockedHeading = heading;
                    headingLocked = true;
                    trackingState = String.format("ON TARGET (tx=%.1f°)", tx);
                    turretPower = 0;
                }

                // Heading lock: counter-rotate turret when chassis spins
                if (headingLocked) {
                    headingError = heading - lockedHeading;
                    while (headingError > 180) headingError -= 360;
                    while (headingError < -180) headingError += 360;
                    turretPower += headingError * K_HL;
                    turretPower = Math.max(-MAX_POWER, Math.min(MAX_POWER, turretPower));
                }

            } else if (headingLocked) {
                // No target but we had one — heading lock holds aim
                headingError = heading - lockedHeading;
                while (headingError > 180) headingError -= 360;
                while (headingError < -180) headingError += 360;
                turretPower = headingError * K_HL;
                turretPower = Math.max(-MAX_POWER, Math.min(MAX_POWER, turretPower));
                trackingState = String.format("HEADING LOCK (err=%.1f°)", headingError);
            }

            // === HARD LIMIT ENFORCEMENT ===
            String limitState = "OK";

            // Hard stops: kill power at or past limits
            if (turretDeg >= MAX_DEG && turretPower > 0) {
                turretPower = 0;
                limitState = "AT MAX LIMIT";
            }
            if (turretDeg <= MIN_DEG && turretPower < 0) {
                turretPower = 0;
                limitState = "AT MIN LIMIT";
            }

            // Slow zones: ramp down approaching limits
            if (turretPower > 0 && turretDeg > MAX_DEG - SLOW_ZONE_DEG) {
                double scale = (MAX_DEG - turretDeg) / SLOW_ZONE_DEG;
                turretPower *= Math.max(0.0, Math.min(1.0, scale));
                limitState = String.format("SLOW (R) %.0f%%", scale * 100);
            }
            if (turretPower < 0 && turretDeg < MIN_DEG + SLOW_ZONE_DEG) {
                double scale = (turretDeg - MIN_DEG) / SLOW_ZONE_DEG;
                turretPower *= Math.max(0.0, Math.min(1.0, scale));
                limitState = String.format("SLOW (L) %.0f%%", scale * 100);
            }

            // Overshoot recovery: if past limit, actively push back
            if (turretDeg > MAX_DEG) {
                turretPower = -0.15;
                limitState = "OVERSHOOT MAX — RECOVERING";
            }
            if (turretDeg < MIN_DEG) {
                turretPower = 0.15;
                limitState = "OVERSHOOT MIN — RECOVERING";
            }

            turret.setPower(turretPower);

            // === TELEMETRY ===
            telemetry.addData("=== TURRET ===", "");
            telemetry.addData("State", trackingState);
            telemetry.addData("Angle", String.format("%.1f° (range: %.0f–%.0f°)", turretDeg, MIN_DEG, MAX_DEG));
            telemetry.addData("Power", String.format("%.3f", turretPower));
            telemetry.addData("Limits", limitState);
            telemetry.addData("Encoder", turretPos);
            telemetry.addData("", "");
            telemetry.addData("=== HEADING LOCK ===", "");
            telemetry.addData("Heading", String.format("%.1f°", heading));
            telemetry.addData("Locked", headingLocked ?
                    String.format("%.1f° (err=%.1f°)", lockedHeading, headingError) : "OFF");
            telemetry.addData("K_HL", K_HL);
            telemetry.addData("", "");
            telemetry.addData("=== TUNE ===", "");
            telemetry.addData("kP", kP);
            telemetry.addData("Deadband", DEADBAND + "°");
            telemetry.addData("Max Power", MAX_POWER);
            telemetry.addData("", "");
            telemetry.addData("=== LIMELIGHT ===", "");
            telemetry.addData("Blue Goal", seesGoal ? "YES" : "NO");
            displayTelemetry(this);
            telemetry.update();

            // Panels
            telemetryM.debug(String.format("TX=%.1f° | Pwr=%.3f | Hdg=%.1f° | HLerr=%.1f°",
                    seesGoal ? getBlueGoalX() : 0.0, turretPower, heading, headingError));
            telemetryM.update();
        }
    }
}
