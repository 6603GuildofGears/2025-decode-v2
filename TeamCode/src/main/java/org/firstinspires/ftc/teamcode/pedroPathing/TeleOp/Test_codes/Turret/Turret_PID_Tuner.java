package org.firstinspires.ftc.teamcode.pedroPathing.TeleOp.Test_codes.Turret;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AngularVelocity;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;

import static org.firstinspires.ftc.teamcode.pedroPathing.Pipelines.Motor_PipeLine.*;
import static org.firstinspires.ftc.teamcode.pedroPathing.Pipelines.Limelight_Pipeline.*;
import static org.firstinspires.ftc.teamcode.pedroPathing.Pipelines.Servo_Pipeline.*;
import static org.firstinspires.ftc.teamcode.pedroPathing.Pipelines.Sensor.*;
import static org.firstinspires.ftc.teamcode.pedroPathing.TeleOp.TurretConfig.*;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import org.firstinspires.ftc.teamcode.pedroPathing.other.Constants;

/**
 * Turret PID Auto-Tuner (Blue Alliance)
 *
 * Uses the Twiddle (coordinate ascent) algorithm to automatically find
 * optimal PID + rotation compensation constants for turret AprilTag tracking.
 *
 * HOW TO USE:
 *  1. Start the OpMode, drive around with gamepad1
 *  2. Keep the blue goal AprilTag in the Limelight's FOV as much as possible
 *  3. The tuner evaluates tracking error over rolling windows
 *  4. It adjusts one parameter at a time, keeping changes that reduce error
 *  5. Watch telemetry for current best values
 *  6. Press gamepad1.back to FREEZE current best values (stops tuning)
 *  7. Copy the "BEST" values into AprilTagCentererConfig when satisfied
 *
 * Tuned parameters: kP, kI, kD, kRot (IMU yaw-rate feedforward)
 */
@Disabled
@TeleOp(name = "Turret PID Tuner", group = "Testing")
public class Turret_PID_Tuner extends LinearOpMode {

    // --- Turret physical limits ---
    private static final int TURRET_MIN_LIMIT = -45;
    private static final int TURRET_MAX_LIMIT = 860;
    private static final int LIMIT_SLOW_ZONE  = 30;

    // --- Camera constants ---
    private static final double CAMERA_HEIGHT      = 11.125;
    private static final double CAMERA_MOUNT_ANGLE = 22.85; // calibrated
    private static final double TARGET_HEIGHT      = 29.5;

    // --- Odometry constants (local — removed from TurretConfig) ---
    private static final double BLUE_GOAL_X = 9.01;
    private static final double BLUE_GOAL_Y = 28.69;
    private static final double TICKS_PER_TURRET_DEG = 2.64;
    private static final double TURRET_FORWARD_TICKS = 130.5;
    private static final double ODO_AIM_POWER = 0.25;

    // --- Twiddle tuning settings ---
    private static final double EVAL_WINDOW_SEC = 2.5;   // seconds per evaluation window
    private static final double MIN_TRACK_RATIO = 0.60;  // must see target 60% of window to count
    private static final double SETTLE_TIME_SEC = 0.4;   // let system settle after param change
    private static final double TWIDDLE_TOLERANCE = 0.0001; // stop when deltas sum < this

    // --- Tracking settings ---
    private static final double DEADBAND_DEG    = 0.15;   // tighter deadband for precision
    private static final double MAX_TURRET_SPEED = 0.85;  // allow faster slewing
    private static final double MIN_TURRET_POWER = 0.04;  // overcome static friction at small errors
    private static final double SCAN_POWER      = 0.045;  // slow enough for Limelight to catch targets
    private static final double SCAN_RAMP_SEC   = 0.6;    // seconds to ramp up to full scan speed
    private static final double FILTER_ALPHA    = 0.55;   // less lag (was 0.88) — faster reaction
    private static final double D_FILTER_ALPHA  = 0.5;    // less lag on derivative too

    // ========== PID + feedforward state ==========
    private double kP   = 0.01969;
    private double kI   = 0.00083;
    private double kD   = 0.00210;
    private double kRot = 0.00378;  // IMU yaw-rate feedforward gain (power per °/s)

    // Best found so far
    private double bestKP   = 0.01969;
    private double bestKI   = 0.00083;
    private double bestKD   = 0.00210;
    private double bestKRot = 0.00378;
    private double bestError = Double.MAX_VALUE;

    // IMU
    private IMU imu;

    // ========== Twiddle state ==========
    // params[0]=kP, params[1]=kI, params[2]=kD, params[3]=kRot
    private double[] params;
    private double[] deltas;
    private int twiddleIndex = 0;       // which param we're adjusting
    private int twiddlePhase = 0;       // 0=try +delta, 1=try -delta, 2=shrink
    private boolean tuningFrozen = false;
    private int twiddleIterations = 0;

    // ========== Evaluation window ==========
    private double windowErrorSum   = 0.0;
    private int    windowSampleCount = 0;
    private int    windowTrackFrames = 0;
    private int    windowTotalFrames = 0;
    private ElapsedTime windowTimer  = new ElapsedTime();
    private ElapsedTime settleTimer  = new ElapsedTime();
    private boolean settling = false;

    // ========== PID controller state ==========
    private double integral       = 0.0;
    private double previousError  = 0.0;
    private double filteredDeriv  = 0.0;   // low-pass filtered derivative
    private boolean filterInited  = false; // true after first target acquisition
    private ElapsedTime pidTimer  = new ElapsedTime();

    // ========== Evaluation: track both MAE and RMS ==========
    private double windowErrorSqSum = 0.0;  // sum of squared errors for RMS

    @Override
    public void runOpMode() throws InterruptedException {
        intMotors(this);
        initLimelight(this);
        initSensors(this);
        intServos(this);

        // Initialize IMU for yaw-rate feedforward
        imu = hardwareMap.get(IMU.class, "imu");
        imu.initialize(new IMU.Parameters(new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.UP,
                RevHubOrientationOnRobot.UsbFacingDirection.FORWARD)));
        imu.resetYaw();

        // Odometry for turret field-relative aiming when Limelight loses target
        Follower follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(new Pose(72, 72, 0)); // adjust to actual start position

        turret.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        turret.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        // Initialize twiddle arrays
        params = new double[]{ kP, kI, kD, kRot };
        deltas = new double[]{ 0.004, 0.001, 0.001, 0.001 };

        // Filtered error for smooth tracking
        double filteredError = 0.0;
        ElapsedTime targetLostTimer = new ElapsedTime();

        while (!isStarted() && !isStopRequested()) {
            int pos = turret.getCurrentPosition();
            if (isMagPressed() && pos > -25 && pos < 25) {
                turret.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                turret.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            }
            telemetry.addData("=== TURRET PID TUNER ===", "");
            telemetry.addData("Starting kP", String.format("%.4f", kP));
            telemetry.addData("Starting kI", String.format("%.4f", kI));
            telemetry.addData("Starting kD", String.format("%.4f", kD));
            telemetry.addData("Starting kRot", String.format("%.4f", kRot));
            telemetry.addData("Turret Pos", pos);
            telemetry.addData("Mag Sensor", isMagPressed() ? "ZEROED" : "not pressed");
            telemetry.addData("", "Press START to begin tuning");
            telemetry.update();
            sleep(20);
        }

        waitForStart();

        double gear = 1.25;
        pidTimer.reset();
        windowTimer.reset();
        settleTimer.reset();
        settling = true; // initial settle

        while (opModeIsActive()) {
            // Update odometry every loop for turret field-relative aiming
            follower.update();

            // ========== DRIVE ==========
            double drive  = -gamepad1.left_stick_y;
            double strafe = gamepad1.left_stick_x;
            double turn   = gamepad1.right_stick_x;

            if (Math.abs(drive) > 0.05 || Math.abs(strafe) > 0.05 || Math.abs(turn) > 0.05) {
                double lf = (drive + strafe + turn) * gear;
                double lb = (drive - strafe + turn) * gear;
                double rf = (drive - strafe - turn) * gear;
                double rb = (drive + strafe - turn) * gear;

                double max = Math.max(Math.abs(lf),
                             Math.max(Math.abs(lb),
                             Math.max(Math.abs(rf), Math.abs(rb))));
                if (max > 1.0) { lf /= max; lb /= max; rf /= max; rb /= max; }

                frontLeft.setPower(lf);
                backLeft.setPower(lb);
                frontRight.setPower(rf);
                backRight.setPower(rb);
            } else {
                SetPower(0, 0, 0, 0);
            }

            // ========== FREEZE TOGGLE ==========
            if (gamepad1.back) {
                tuningFrozen = true;
                // Lock in best values
                kP = bestKP; kI = bestKI; kD = bestKD; kRot = bestKRot;
                params[0] = kP; params[1] = kI; params[2] = kD; params[3] = kRot;
            }

            // ========== TURRET TRACKING (PID) ==========
            int turretPosition = turret.getCurrentPosition();
            double turretPower = 0.0;
            boolean hasTarget = hasBlueGoal();

            // IMU yaw-rate feedforward: counter-rotate turret to cancel chassis spin
            double yawRate = imu.getRobotAngularVelocity(AngleUnit.DEGREES).zRotationRate;
            double rotationFF = yawRate * kRot;

            double dt = pidTimer.seconds();
            pidTimer.reset();
            if (dt > 0.5) dt = 0.02; // guard against first-frame spike

            if (hasTarget) {
                double tx = getBlueGoalX();

                // Low-pass filter
                if (!filterInited) {
                    filteredError = tx;
                    previousError = tx;
                    filteredDeriv = 0.0;
                    filterInited = true;
                } else {
                    filteredError = FILTER_ALPHA * tx + (1.0 - FILTER_ALPHA) * filteredError;
                }

                if (Math.abs(filteredError) > DEADBAND_DEG) {
                    // PID calculation
                    double error = filteredError;
                    integral += error * dt;
                    // Anti-windup: clamp integral
                    double maxIntegral = MAX_TURRET_SPEED / (kI == 0 ? 1.0 : Math.max(Math.abs(kI), 0.0001));
                    integral = Math.max(-maxIntegral, Math.min(maxIntegral, integral));

                    // Filtered derivative: low-pass to reject Limelight measurement noise
                    double rawDeriv = (error - previousError) / dt;
                    filteredDeriv = D_FILTER_ALPHA * rawDeriv + (1.0 - D_FILTER_ALPHA) * filteredDeriv;
                    previousError = error;

                    turretPower = (kP * error) + (kI * integral) + (kD * filteredDeriv);

                    // Static friction compensation: ensure minimum power to actually move
                    if (Math.abs(turretPower) > 0.001 && Math.abs(turretPower) < MIN_TURRET_POWER) {
                        turretPower = Math.signum(turretPower) * MIN_TURRET_POWER;
                    }
                } else {
                    turretPower = 0.0;
                    // Quickly decay integral when on target (prevent stale bias)
                    integral *= 0.8;
                    previousError = filteredError;
                }

                // IMU yaw-rate feedforward — always apply full strength
                // (the whole point is to cancel chassis rotation even when centered)
                turretPower += rotationFF;
                turretPower = Math.max(-MAX_TURRET_SPEED, Math.min(MAX_TURRET_SPEED, turretPower));

                targetLostTimer.reset();

                // --- Collect evaluation samples ---
                if (!settling) {
                    windowErrorSum += Math.abs(tx);
                    windowErrorSqSum += tx * tx;  // for RMS
                    windowSampleCount++;
                    windowTrackFrames++;
                }
            } else {
                // No target — use odometry to aim turret at predicted goal angle
                integral = 0.0;
                previousError = 0.0;
                filteredDeriv = 0.0;
                filterInited = false;

                // Start with yaw-rate compensation
                turretPower = rotationFF;

                double timeLost = targetLostTimer.seconds();
                if (timeLost > 0.15) {
                    // Odometry-guided aim: calculate field-relative angle to goal
                    Pose pose = follower.getPose();
                    double dx = BLUE_GOAL_X - pose.getX();
                    double dy = BLUE_GOAL_Y - pose.getY();
                    double fieldAngleRad = Math.atan2(dy, dx);

                    // Robot-relative angle (subtract robot heading)
                    double robotRelRad = fieldAngleRad - pose.getHeading();
                    robotRelRad = Math.atan2(Math.sin(robotRelRad), Math.cos(robotRelRad));
                    double robotRelDeg = Math.toDegrees(robotRelRad);

                    // Convert to target turret encoder ticks
                    double targetTicks = TURRET_FORWARD_TICKS + (robotRelDeg * TICKS_PER_TURRET_DEG);
                    targetTicks = Math.max(TURRET_MIN_LIMIT, Math.min(TURRET_MAX_LIMIT, targetTicks));

                    double tickError = targetTicks - turretPosition;

                    if (Math.abs(tickError) > 3) {
                        // Proportional slew toward predicted position
                        turretPower = Math.signum(tickError) * Math.min(ODO_AIM_POWER,
                                Math.abs(tickError) * 0.005);
                        turretPower += rotationFF;
                    }

                    turretPower = Math.max(-MAX_TURRET_SPEED, Math.min(MAX_TURRET_SPEED, turretPower));
                } else {
                    // Brief grace period — just hold with yaw compensation
                    turretPower = Math.max(-MAX_TURRET_SPEED, Math.min(MAX_TURRET_SPEED, turretPower));
                }
            }

            // Count total frames for tracking ratio
            if (!settling) {
                windowTotalFrames++;
            }

            // ========== TURRET LIMITS (no whip) ==========
            turretPosition = turret.getCurrentPosition();
            if (turretPosition >= TURRET_MAX_LIMIT && turretPower > 0) {
                turretPower = 0.0;
            } else if (turretPower > 0 && turretPosition > TURRET_MAX_LIMIT - LIMIT_SLOW_ZONE) {
                double s = (double)(TURRET_MAX_LIMIT - turretPosition) / LIMIT_SLOW_ZONE;
                turretPower *= Math.max(0.0, Math.min(1.0, s));
            }
            if (turretPosition <= TURRET_MIN_LIMIT && turretPower < 0) {
                turretPower = 0.0;
            } else if (turretPower < 0 && turretPosition < TURRET_MIN_LIMIT + LIMIT_SLOW_ZONE) {
                double s = (double)(turretPosition - TURRET_MIN_LIMIT) / LIMIT_SLOW_ZONE;
                turretPower *= Math.max(0.0, Math.min(1.0, s));
            }

            turret.setPower(turretPower);

            // ========== TWIDDLE EVALUATION ==========
            if (settling && settleTimer.seconds() >= SETTLE_TIME_SEC) {
                settling = false;
                resetWindow();
            }

            if (!settling && !tuningFrozen && windowTimer.seconds() >= EVAL_WINDOW_SEC) {
                evaluateAndTwiddle();
            }

            // ========== TELEMETRY ==========
            String paramNames = "kP kI kD kRot";
            String[] pNames = {"kP", "kI", "kD", "kRot"};
            String tuningParam = twiddleIndex < pNames.length ? pNames[twiddleIndex] : "?";

            telemetry.addData("=== TURRET PID TUNER ===", tuningFrozen ? "FROZEN" : "TUNING");
            telemetry.addData("Iteration", twiddleIterations);
            telemetry.addData("Tuning Param", tuningParam + " (phase " + twiddlePhase + ")");
            telemetry.addData("", "--- Current ---");
            telemetry.addData("kP", String.format("%.5f", kP));
            telemetry.addData("kI", String.format("%.5f", kI));
            telemetry.addData("kD", String.format("%.5f", kD));
            telemetry.addData("kRot", String.format("%.5f", kRot));
            telemetry.addData("", "--- Best (err: " + String.format("%.3f", bestError) + "°) ---");
            telemetry.addData("best kP", String.format("%.5f", bestKP));
            telemetry.addData("best kI", String.format("%.5f", bestKI));
            telemetry.addData("best kD", String.format("%.5f", bestKD));
            telemetry.addData("best kRot", String.format("%.5f", bestKRot));
            telemetry.addData("", "--- Deltas ---");
            telemetry.addData("dP/dI/dD/dR",
                    String.format("%.5f / %.5f / %.5f / %.4f",
                            deltas[0], deltas[1], deltas[2], deltas[3]));
            double deltaSum = deltas[0] + deltas[1] + deltas[2] + deltas[3];
            telemetry.addData("Delta Sum", String.format("%.6f (tol: %.4f)", deltaSum, TWIDDLE_TOLERANCE));
            telemetry.addData("", "--- Live ---");
            telemetry.addData("Yaw Rate", String.format("%.1f °/s", yawRate));
            telemetry.addData("Target", hasTarget ? "BLUE" : "ODO AIM");
            telemetry.addData("TX", hasTarget ? String.format("%.2f°", getBlueGoalX()) : "--");
            Pose tPose = follower.getPose();
            telemetry.addData("Odo Pose", String.format("(%.1f, %.1f) %.1f°",
                    tPose.getX(), tPose.getY(), Math.toDegrees(tPose.getHeading())));
            telemetry.addData("Filtered Err", String.format("%.2f°", filteredError));
            telemetry.addData("Turret Power", String.format("%.3f", turretPower));
            telemetry.addData("Turret Pos", turretPosition);
            double trackRatio = windowTotalFrames > 0 ? (double) windowTrackFrames / windowTotalFrames : 0;
            telemetry.addData("Window Track%", String.format("%.0f%%", trackRatio * 100));
            telemetry.addData("Window MAE", windowSampleCount > 0
                    ? String.format("%.3f°", windowErrorSum / windowSampleCount) : "--");
            telemetry.addData("Window RMS", windowSampleCount > 0
                    ? String.format("%.3f°", Math.sqrt(windowErrorSqSum / windowSampleCount)) : "--");
            telemetry.addData("", "gamepad1.back = FREEZE best values");
            telemetry.update();
        }
    }

    // ========== TWIDDLE LOGIC ==========

    private void evaluateAndTwiddle() {
        double trackRatio = windowTotalFrames > 0
                ? (double) windowTrackFrames / windowTotalFrames : 0;

        // Only evaluate if we tracked enough of the window
        if (trackRatio < MIN_TRACK_RATIO || windowSampleCount < 10) {
            // Not enough data — retry same config
            beginSettle();
            return;
        }

        double currentMAE = windowErrorSum / windowSampleCount;
        double currentRMS = Math.sqrt(windowErrorSqSum / windowSampleCount);
        // Blend MAE + RMS: penalizes oscillation more than steady-state offset
        double currentError = 0.6 * currentMAE + 0.4 * currentRMS;

        // Check if delta sum is below tolerance (tuning converged)
        double deltaSum = 0;
        for (double d : deltas) deltaSum += d;
        if (deltaSum < TWIDDLE_TOLERANCE) {
            tuningFrozen = true;
            kP = bestKP; kI = bestKI; kD = bestKD; kRot = bestKRot;
            return;
        }

        switch (twiddlePhase) {
            case 0: // We just tried +delta — evaluate
                if (currentError < bestError) {
                    // Improvement! Keep it, grow delta
                    bestError = currentError;
                    saveBest();
                    deltas[twiddleIndex] *= 1.1;
                    advanceParam();
                    // Apply +delta to next param
                    params[twiddleIndex] += deltas[twiddleIndex];
                    clampParams();
                    applyParams();
                    twiddlePhase = 0;
                } else {
                    // No improvement — try -2*delta (undo + go other way)
                    params[twiddleIndex] -= 2.0 * deltas[twiddleIndex];
                    clampParams();
                    applyParams();
                    twiddlePhase = 1;
                }
                break;

            case 1: // We just tried -delta — evaluate
                if (currentError < bestError) {
                    // Improvement! Keep it, grow delta
                    bestError = currentError;
                    saveBest();
                    deltas[twiddleIndex] *= 1.1;
                    advanceParam();
                    params[twiddleIndex] += deltas[twiddleIndex];
                    clampParams();
                    applyParams();
                    twiddlePhase = 0;
                } else {
                    // Neither direction helped — revert, shrink delta
                    params[twiddleIndex] += deltas[twiddleIndex]; // revert to original
                    clampParams();
                    deltas[twiddleIndex] *= 0.75;
                    applyParams();
                    advanceParam();
                    params[twiddleIndex] += deltas[twiddleIndex];
                    clampParams();
                    applyParams();
                    twiddlePhase = 0;
                }
                break;
        }

        twiddleIterations++;
        integral = 0.0; // reset PID state after param change
        previousError = 0.0;
        filteredDeriv = 0.0;
        filterInited = false;
        beginSettle();
    }

    private void advanceParam() {
        twiddleIndex = (twiddleIndex + 1) % params.length;
    }

    private void saveBest() {
        bestKP = kP;
        bestKI = kI;
        bestKD = kD;
        bestKRot = kRot;
    }

    private void applyParams() {
        // Enforce non-negative for P, I, D
        if (params[0] < 0.001) params[0] = 0.001; // kP minimum
        if (params[1] < 0.0) params[1] = 0.0;     // kI minimum
        if (params[2] < 0.0) params[2] = 0.0;     // kD minimum
        if (params[3] < 0.0) params[3] = 0.0;     // kRot minimum
        if (params[3] > 0.02) params[3] = 0.02;   // kRot maximum

        kP   = params[0];
        kI   = params[1];
        kD   = params[2];
        kRot = params[3];
    }

    private void clampParams() {
        if (params[0] < 0.001) params[0] = 0.001;
        if (params[1] < 0.0)   params[1] = 0.0;
        if (params[2] < 0.0)   params[2] = 0.0;
        if (params[3] < 0.0)   params[3] = 0.0;
        if (params[3] > 0.02)  params[3] = 0.02;
    }

    private void resetWindow() {
        windowErrorSum    = 0.0;
        windowErrorSqSum  = 0.0;
        windowSampleCount = 0;
        windowTrackFrames = 0;
        windowTotalFrames = 0;
        windowTimer.reset();
    }

    private void beginSettle() {
        settling = true;
        settleTimer.reset();
        resetWindow();
    }
}
