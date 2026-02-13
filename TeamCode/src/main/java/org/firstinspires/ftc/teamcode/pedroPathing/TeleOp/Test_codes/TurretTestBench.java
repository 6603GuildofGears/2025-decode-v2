package org.firstinspires.ftc.teamcode.pedroPathing.TeleOp.Test_codes;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;

import org.firstinspires.ftc.teamcode.pedroPathing.other.Constants;
import org.firstinspires.ftc.teamcode.pedroPathing.Pipelines.TurretTest;

import static org.firstinspires.ftc.teamcode.pedroPathing.Pipelines.Motor_PipeLine.*;
import static org.firstinspires.ftc.teamcode.pedroPathing.Pipelines.Limelight_Pipeline.*;
import static org.firstinspires.ftc.teamcode.pedroPathing.Pipelines.Sensor.*;

/**
 * TurretTest bench OpMode.
 *
 * CONTROLS (gamepad2):
 *   A (toggle)        — PID on/off (manual stick control when off)
 *   B (toggle)        — Vision tracking on/off (feeds Limelight to PID)
 *   DPAD UP/DOWN      — Step target angle ±30° (only when PID on + vision off)
 *   LEFT STICK X      — Manual turret power (only when PID off)
 *   RIGHT BUMPER      — Reset encoder via magnet (hold during init)
 *   BACK              — Clear field-angle lock
 *
 * CONTROLS (gamepad1):
 *   Left stick / Right stick — Drive (mecanum)
 */
@TeleOp(name = "Turret Test Bench", group = "Testing")
public class TurretTestBench extends OpMode {

    private TurretTest turretCtrl;
    private Follower   follower;

    // Toggle states
    private boolean pidEnabled    = true;
    private boolean visionEnabled = true;

    // Angle stepping
    private double targetAngleDeg = 0;       // robot-relative target for stepping mode
    private static final double STEP_DEG = 30.0;

    // Edge detectors (previous-frame button states)
    private boolean prevA     = false;
    private boolean prevB     = false;
    private boolean prevDUp   = false;
    private boolean prevDDown = false;
    private boolean prevBack  = false;
    private boolean prevMag   = false;   // magnet edge detector for active zeroing

    @Override
    public void init() {
        // Hardware init
        intMotors(this);
        initLimelight(this);
        initSensors(this);

        // Odometry (for heading) — create FIRST, before turret encoder setup,
        // because createFollower() may reset motor encoders on the hub.
        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(new Pose(72, 72, 0));

        // Now set up turret encoder AFTER follower, so it can't be clobbered.
        turret.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        turret.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        turret.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // Turret subsystem
        turretCtrl = new TurretTest(turret);

        telemetry.addData("Status", "Move turret to magnet to zero encoder");
        telemetry.addData("Controls", "A=PID  B=Vision  DPad=Step  LStickX=Manual");
        telemetry.update();
    }

    @Override
    public void init_loop() {
        // Only zero encoder when magnet is physically triggered
        if (isMagPressed()) {
            turret.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            turret.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            telemetry.addData("Encoder", "ZEROED via magnet");
        } else {
            telemetry.addData("Encoder", turret.getCurrentPosition());
        }
        telemetry.addData("Mag", isMagPressed() ? "PRESSED" : "---");
        telemetry.addData("Turret Angle", String.format("%.1f°", turretCtrl.getTurretAngle()));
        telemetry.update();
    }

    @Override
    public void start() {
        turretCtrl.reset();
        follower.startTeleopDrive();
        telemetry.setAutoClear(false);
    }

    @Override
    public void loop() {
        telemetry.clearAll();

        // ==============================================================
        //  Odometry + Limelight update (poll ONCE per loop)
        // ==============================================================
        follower.update();
        pollOnce();   // cache Limelight result for this entire loop iteration
        double robotHeadingDeg = Math.toDegrees(follower.getPose().getHeading());

        // ==============================================================
        //  Active magnet zeroing (rising-edge only)
        // ==============================================================
        boolean magNow = isMagPressed();
        if (magNow && !prevMag) {
            turretCtrl.zeroEncoder();
            targetAngleDeg = 0;   // reset step target — coordinate system just changed
        }
        prevMag = magNow;

        // ==============================================================
        //  Button edge detection
        // ==============================================================
        boolean aPressed     = gamepad2.a     && !prevA;
        boolean bPressed     = gamepad2.b     && !prevB;
        boolean dUpPressed   = gamepad2.dpad_up   && !prevDUp;
        boolean dDownPressed = gamepad2.dpad_down && !prevDDown;
        boolean backPressed  = gamepad2.back  && !prevBack;

        prevA     = gamepad2.a;
        prevB     = gamepad2.b;
        prevDUp   = gamepad2.dpad_up;
        prevDDown = gamepad2.dpad_down;
        prevBack  = gamepad2.back;

        // --- Toggle PID ---
        if (aPressed) {
            pidEnabled = !pidEnabled;
            if (!pidEnabled) {
                turret.setPower(0);  // stop motor when disabling
            }
        }

        // --- Toggle Vision ---
        if (bPressed) {
            visionEnabled = !visionEnabled;
        }

        // --- Clear lock ---
        if (backPressed) {
            turretCtrl.clearLock();
        }

        // --- Angle stepping (PID on + vision off) ---
        if (pidEnabled && !visionEnabled) {
            if (dUpPressed)   targetAngleDeg += STEP_DEG;
            if (dDownPressed) targetAngleDeg -= STEP_DEG;
        }

        // ==============================================================
        //  Drive (gamepad1, simple mecanum)
        // ==============================================================
        double drive  =  gamepad1.left_stick_y;
        double strafe = -gamepad1.left_stick_x;
        double turn   = -gamepad1.right_stick_x;
        double gear   = 0.8;

        double lf = (drive + strafe + turn) * gear;
        double lb = (drive - strafe + turn) * gear;
        double rf = (drive - strafe - turn) * gear;
        double rb = (drive + strafe - turn) * gear;

        double maxDrive = Math.max(1.0, Math.max(Math.abs(lf),
                          Math.max(Math.abs(lb),
                          Math.max(Math.abs(rf), Math.abs(rb)))));
        frontLeft.setPower(lf / maxDrive);
        backLeft.setPower(lb / maxDrive);
        frontRight.setPower(rf / maxDrive);
        backRight.setPower(rb / maxDrive);

        // ==============================================================
        //  Turret control
        // ==============================================================
        if (!pidEnabled) {
            // --- Manual mode: left stick X drives turret directly ---
            double manualPwr = gamepad2.left_stick_x * 0.4;
            manualPwr = turretCtrl.limitPower(manualPwr);  // enforce soft-limits
            turret.setPower(manualPwr);

        } else if (visionEnabled) {
            // --- Vision tracking: feed Limelight data to TurretTest ---
            boolean hasTarget = hasBlueGoal();
            double  tx        = hasTarget ? getBlueGoalX() : 0;

            turretCtrl.setInputs(hasTarget, tx, robotHeadingDeg);
            turretCtrl.update();

        } else {
            // --- Angle stepping: feed a fake "target" error from encoder position ---
            // We want turretAngle to reach targetAngleDeg.
            // Compute error = target - current, feed as if it were tx
            // (no vision, no lock — just raw PID to an angle)
            double currentAngle = turretCtrl.getTurretAngle();
            double stepError    = targetAngleDeg - currentAngle;

            // Feed as TRACKING with synthetic tx = error, heading unused
            turretCtrl.setInputs(true, stepError, robotHeadingDeg);
            turretCtrl.update();
        }

        // ==============================================================
        //  Telemetry
        // ==============================================================

        telemetry.addData("=== TURRET TEST BENCH ===", "");
        telemetry.addData("Mode",
                !pidEnabled ? "MANUAL (LStick X)" :
                visionEnabled ? "VISION TRACK" : "ANGLE STEP");

        telemetry.addLine();
        telemetry.addData("State",       turretCtrl.getState());
        telemetry.addData("On Target",   turretCtrl.isOnTarget());
        telemetry.addData("Has Lock",    turretCtrl.hasLock());

        telemetry.addLine();
        telemetry.addData("Turret Angle", String.format("%.1f°", turretCtrl.getTurretAngle()));
        telemetry.addData("Target Angle", String.format("%.1f°", targetAngleDeg));
        telemetry.addData("Error",        String.format("%.2f°", turretCtrl.getError()));
        telemetry.addData("Power",        String.format("%.3f",  turretCtrl.getPower()));

        telemetry.addLine();
        telemetry.addData("P term", String.format("%.4f", turretCtrl.getPTerm()));
        telemetry.addData("I term", String.format("%.4f", turretCtrl.getITerm()));
        telemetry.addData("D term", String.format("%.4f", turretCtrl.getDTerm()));
        telemetry.addData("Integral", String.format("%.3f", turretCtrl.getIntegral()));

        telemetry.addLine();
        telemetry.addData("Encoder Raw",    turretCtrl.getEncoderRaw());
        telemetry.addData("Robot Heading",  String.format("%.1f°", robotHeadingDeg));
        telemetry.addData("Locked Angle",   String.format("%.1f°", turretCtrl.getLockedAngle()));
        telemetry.addData("Loop Hz",        String.format("%.0f", turretCtrl.getLoopHz()));

        telemetry.addLine();
        telemetry.addData("Mag Sensor",  isMagPressed() ? "PRESSED (zeroed!)" : "---");
        telemetry.addData("LL Target",   hasBlueGoal() ? String.format("TX=%.2f°", getBlueGoalX()) : "NONE");

        telemetry.addLine();
        telemetry.addData("[A] PID",      pidEnabled  ? "ON" : "OFF");
        telemetry.addData("[B] Vision",   visionEnabled ? "ON" : "OFF");
        telemetry.addData("[DPad] Step",  pidEnabled && !visionEnabled ? "ACTIVE (±30°)" : "---");
        telemetry.addData("[Back]",       "Clear Lock");

        telemetry.update();
    }

    @Override
    public void stop() {
        turret.setPower(0);
    }
}
