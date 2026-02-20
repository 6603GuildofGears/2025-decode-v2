package org.firstinspires.ftc.teamcode.pedroPathing.TeleOp.Test_codes.Turret;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import static org.firstinspires.ftc.teamcode.pedroPathing.Pipelines.Motor_PipeLine.*;
import static org.firstinspires.ftc.teamcode.pedroPathing.Pipelines.Limelight_Pipeline.*;
import static org.firstinspires.ftc.teamcode.pedroPathing.TeleOp.TurretConfig.*;
import static org.firstinspires.ftc.teamcode.pedroPathing.TeleOp.TurretTunerConfig.*;

@Disabled
@TeleOp(name = "Turret PID Assistant", group = "Testing")
public class Turret_PID_Assistant extends LinearOpMode {

    private static final double TURRET_GEAR_RATIO = 131.0 / 20.0;
    private static final int TURRET_MIN_LIMIT = -275;
    private static final int TURRET_MAX_LIMIT = 630;
    private static final double STEP_THRESHOLD = 3.0; // degrees
    private static final double SETTLE_HOLD_SECONDS = 0.5;

    @Override
    public void runOpMode() throws InterruptedException {
        intMotors(this);
        initLimelight(this);

        // Reset turret encoder to 0 at current position (center manually before init)
        turret.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        turret.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        ElapsedTime pidTimer = new ElapsedTime();
        ElapsedTime testTimer = new ElapsedTime();
        ElapsedTime settleTimer = new ElapsedTime();

        double lastError = 0.0;
        double integratedError = 0.0;
        double filteredError = 0.0;

        boolean runningTest = false;
        boolean inBand = false;
        boolean crossedZero = false;
        boolean resultsReady = false;
        int zeroCrossings = 0;

        double startError = 0.0;
        double peakError = 0.0;
        double peakOpposite = 0.0;
        double lastErrorSign = 0.0;
        double settleTime = 0.0;
        double testDuration = 0.0;
        double lastTuneTime = 0.0;

        limelight.start();
        limelight.setPollRateHz(100); // 100Hz for smooth tracking

        double driveGear = 1.0; // drive speed modifier

        waitForStart();
        while (opModeIsActive()) {
            boolean a = gamepad1.a;
            boolean b = gamepad1.b;
            boolean x = gamepad1.x;

            double responseTime = Math.max(0.5, RESPONSE_TIME_SECONDS);
            double speedBias = Math.max(0.0, Math.min(1.0, SPEED_PRIORITY));
            double responseFactor = Math.max(0.5, Math.min(2.0, 2.0 / responseTime));
            double kpUpFactor = 1.0 + (0.05 * responseFactor) + (0.05 * speedBias);
            double kpDownFactor = 1.0 - (0.03 * responseFactor) - (0.03 * speedBias);
            double kdUpFactor = 1.0 + (0.04 * responseFactor) + (0.04 * speedBias);
            double kiUpFactor = 1.0 + (0.03 * responseFactor);
            double alphaUpFactor = 1.0 + (0.05 * responseFactor);
            double alphaDownFactor = 1.0 - (0.04 * responseFactor);
            double deadbandDownFactor = 1.0 - (0.05 * responseFactor);
            double deadbandUpFactor = 1.0 + (0.04 * responseFactor);
            double tuneIntervalSeconds = Math.max(0.5, responseTime);

            // Drive code (mecanum)
            double driveX = -gamepad1.left_stick_x;
            double driveTurn = gamepad1.left_stick_y;
            double driveY = -gamepad1.right_stick_x;

            if (Math.abs(driveX) > 0.05 || Math.abs(driveY) > 0.05 || Math.abs(driveTurn) > 0.05) {
                double r = Math.hypot(driveX, driveY);
                double robotAngle = Math.atan2(driveY, driveX) - Math.PI / 4;
                double v1 = r * Math.cos(robotAngle) + driveTurn * driveGear; // lf
                double v2 = r * Math.sin(robotAngle) - driveTurn * driveGear; // rf
                double v3 = r * Math.sin(robotAngle) + driveTurn * driveGear; // lb
                double v4 = r * Math.cos(robotAngle) - driveTurn * driveGear; // rb
                SetPower(v1, v3, v2, v4);
            } else {
                SetPower(0, 0, 0, 0);
            }

            // Limelight target error (blue goal)
            double tx = 0.0;
            boolean hasGoal = hasBlueGoal();
            if (hasGoal) {
                tx = getBlueGoalX();
            }

            // Initialize filter on first reading
            if (filteredError == 0 && lastError == 0) {
                filteredError = tx;
            } else {
                filteredError = FILTER_ALPHA * tx + (1 - FILTER_ALPHA) * filteredError;
            }

            // PID control (same structure as main opmode)
            double turretPower = 0.0;
            if (hasGoal) {
                if (Math.abs(filteredError) > TURRET_DEADBAND) {
                    double compensatedError = filteredError * TURRET_GEAR_RATIO;
                    double pTerm = compensatedError * KP_TURRET;

                    double dt = pidTimer.seconds();
                    double dTerm = 0.0;
                    if (dt > 0.01 && dt < 1.0) {
                        double errorChange = (filteredError - lastError) / dt;
                        dTerm = errorChange * KD_TURRET;
                    }

                    integratedError += filteredError * dt;
                    integratedError = Math.max(-0.2, Math.min(0.2, integratedError));
                    double iTerm = integratedError * KI_TURRET;

                    turretPower = pTerm + iTerm + dTerm;
                    turretPower = Math.max(-MAX_TURRET_SPEED, Math.min(MAX_TURRET_SPEED, turretPower));

                    lastError = filteredError;
                    pidTimer.reset();
                } else {
                    turretPower = 0.0;
                    lastError = filteredError;
                }
            } else {
                lastError = 0.0;
                integratedError = 0.0;
                filteredError = 0.0;
                turretPower = 0.0;
            }

            // Apply safety limits
            int turretPosition = turret.getCurrentPosition();
            if (turretPosition <= TURRET_MIN_LIMIT && turretPower < 0) {
                turretPower = 0;
            }
            if (turretPosition >= TURRET_MAX_LIMIT && turretPower > 0) {
                turretPower = 0;
            }
            turret.setPower(turretPower);

            // Test control
            if (b) {
                runningTest = false;
                resultsReady = false;
                inBand = false;
                crossedZero = false;
                zeroCrossings = 0;
                startError = 0.0;
                peakError = 0.0;
                peakOpposite = 0.0;
                lastErrorSign = 0.0;
                settleTime = 0.0;
                testDuration = 0.0;
                lastTuneTime = 0.0;
            }

            if (a && hasGoal && !runningTest) {
                runningTest = true;
                resultsReady = false;
                inBand = false;
                crossedZero = false;
                zeroCrossings = 0;
                startError = tx;
                peakError = Math.abs(startError);
                peakOpposite = 0.0;
                lastErrorSign = Math.signum(startError);
                testTimer.reset();
                settleTimer.reset();
                lastTuneTime = 0.0;
            }

            if (x) {
                AUTO_TUNE_ENABLED = !AUTO_TUNE_ENABLED;
            }

            if (runningTest) {
                double error = tx;
                double absErr = Math.abs(error);
                testDuration = testTimer.seconds();

                if (!crossedZero) {
                    if (Math.signum(error) != 0 && Math.signum(error) != lastErrorSign) {
                        crossedZero = true;
                        zeroCrossings++;
                        peakOpposite = absErr;
                    } else {
                        peakError = Math.max(peakError, absErr);
                    }
                } else {
                    if (Math.signum(error) != 0 && Math.signum(error) != lastErrorSign) {
                        zeroCrossings++;
                        lastErrorSign = Math.signum(error);
                    }
                    peakOpposite = Math.max(peakOpposite, absErr);
                }

                if (absErr <= TURRET_DEADBAND) {
                    if (!inBand) {
                        inBand = true;
                        settleTimer.reset();
                    }
                } else {
                    inBand = false;
                }
                if (AUTO_TUNE_ENABLED && testTimer.seconds() - lastTuneTime >= tuneIntervalSeconds) {
                    if (!crossedZero && peakError > STEP_THRESHOLD) {
                        KP_TURRET = Math.min(KP_MAX, Math.max(0.0, KP_TURRET * kpUpFactor));
                        FILTER_ALPHA = Math.min(ALPHA_MAX, Math.max(ALPHA_MIN, FILTER_ALPHA * alphaUpFactor));
                        TURRET_DEADBAND = Math.max(DEADBAND_MIN, Math.min(DEADBAND_MAX, TURRET_DEADBAND * deadbandDownFactor));
                    } else if (peakOpposite > TURRET_DEADBAND * 2) {
                        KP_TURRET = Math.max(0.0, KP_TURRET * kpDownFactor);
                        KD_TURRET = Math.min(KD_MAX, Math.max(0.0, KD_TURRET * kdUpFactor));
                        FILTER_ALPHA = Math.max(ALPHA_MIN, Math.min(ALPHA_MAX, FILTER_ALPHA * alphaDownFactor));
                        TURRET_DEADBAND = Math.max(DEADBAND_MIN, Math.min(DEADBAND_MAX, TURRET_DEADBAND * deadbandUpFactor));
                    }

                    if (zeroCrossings >= 3) {
                        KP_TURRET = Math.max(0.0, KP_TURRET * kpDownFactor);
                        KD_TURRET = Math.min(KD_MAX, Math.max(0.0, KD_TURRET * kdUpFactor));
                        FILTER_ALPHA = Math.max(ALPHA_MIN, Math.min(ALPHA_MAX, FILTER_ALPHA * alphaDownFactor));
                    } else if (Math.abs(tx) > TURRET_DEADBAND && testDuration > 2.0) {
                        KI_TURRET = Math.min(KI_MAX, Math.max(0.0, KI_TURRET * kiUpFactor));
                    }

                    lastTuneTime = testTimer.seconds();
                    peakError = Math.abs(error);
                    peakOpposite = 0.0;
                    zeroCrossings = 0;
                    crossedZero = false;
                    lastErrorSign = Math.signum(error);
                }
            }

            // Suggestions (numeric)
            String suggestion1 = "";
            String suggestion2 = "";

            if (resultsReady || runningTest) {
                if (!crossedZero && peakError > STEP_THRESHOLD) {
                    double kpUp = KP_TURRET * kpUpFactor;
                    suggestion1 = String.format("Sluggish: KP +%.0f%% → %.4f", (kpUpFactor - 1.0) * 100.0, kpUp);
                } else if (peakOpposite > TURRET_DEADBAND * 2) {
                    double kpDown = KP_TURRET * kpDownFactor;
                    double kdUp = KD_TURRET * kdUpFactor;
                    suggestion1 = String.format("Overshoot: KP -%.0f%% → %.4f or KD +%.0f%% → %.4f", (1.0 - kpDownFactor) * 100.0, kpDown, (kdUpFactor - 1.0) * 100.0, kdUp);
                }

                if (zeroCrossings >= 3) {
                    double kpDown = KP_TURRET * kpDownFactor;
                    double kdUp = KD_TURRET * kdUpFactor;
                    suggestion2 = String.format("Oscillation: KP -%.0f%% → %.4f or KD +%.0f%% → %.4f", (1.0 - kpDownFactor) * 100.0, kpDown, (kdUpFactor - 1.0) * 100.0, kdUp);
                } else if (Math.abs(tx) > TURRET_DEADBAND && testDuration > 2.0) {
                    double kiUp = KI_TURRET * kiUpFactor;
                    suggestion2 = String.format("Steady error: KI +%.0f%% → %.5f", (kiUpFactor - 1.0) * 100.0, kiUp);
                }
            }

            telemetry.addData("=== TURRET PID ASSISTANT ===", "");
            telemetry.addData("Goal Visible", hasGoal ? "YES" : "NO");
            telemetry.addData("TX Error", String.format("%.2f°", tx));
            telemetry.addData("Filtered Error", String.format("%.2f°", filteredError));
            telemetry.addData("Turret Pos", turretPosition);
            telemetry.addData("Turret Power", String.format("%.2f", turretPower));
            telemetry.addData("", "");
            telemetry.addData("Instructions", "Aim at BLUE goal, press A to test, B to reset");
            telemetry.addData("Auto Tune", AUTO_TUNE_ENABLED ? "ON (X to toggle)" : "OFF (X to toggle)");
            telemetry.addData("Response Time", String.format("%.2fs", responseTime));
            telemetry.addData("Tune Interval", String.format("%.1fs", tuneIntervalSeconds));
            telemetry.addData("Speed Priority", String.format("%.2f", SPEED_PRIORITY));
            telemetry.addData("Filter Alpha", String.format("%.2f", FILTER_ALPHA));
            telemetry.addData("Deadband", String.format("%.2f", TURRET_DEADBAND));

            if (runningTest) {
                telemetry.addData("Test", "RUNNING");
                telemetry.addData("Peak Error", String.format("%.2f°", peakError));
                telemetry.addData("Peak Opposite", String.format("%.2f°", peakOpposite));
                telemetry.addData("Zero Crossings", zeroCrossings);
                telemetry.addData("Time", String.format("%.2fs", testTimer.seconds()));
                if (!suggestion1.isEmpty()) telemetry.addData("Suggestion", suggestion1);
                if (!suggestion2.isEmpty()) telemetry.addData("Suggestion 2", suggestion2);
            } else if (resultsReady) {
                telemetry.addData("Test", "DONE");
                telemetry.addData("Peak Error", String.format("%.2f°", peakError));
                telemetry.addData("Peak Opposite", String.format("%.2f°", peakOpposite));
                telemetry.addData("Zero Crossings", zeroCrossings);
                telemetry.addData("Settle Time", settleTime < 0 ? "NO SETTLE" : String.format("%.2fs", settleTime));
                if (!suggestion1.isEmpty()) telemetry.addData("Suggestion", suggestion1);
                if (!suggestion2.isEmpty()) telemetry.addData("Suggestion 2", suggestion2);
            } else {
                telemetry.addData("Test", "IDLE");
            }

            telemetry.addData("", "");
            telemetry.addData("KP", String.format("%.4f", KP_TURRET));
            telemetry.addData("KI", String.format("%.5f", KI_TURRET));
            telemetry.addData("KD", String.format("%.4f", KD_TURRET));
            telemetry.update();
        }
    }
}
