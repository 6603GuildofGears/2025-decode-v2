package org.firstinspires.ftc.teamcode.pedroPathing.TeleOp.Test_codes;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import static org.firstinspires.ftc.teamcode.pedroPathing.Pipelines.Motor_PipeLine.*;
import static org.firstinspires.ftc.teamcode.pedroPathing.Pipelines.Limelight_Pipeline.*;
import static org.firstinspires.ftc.teamcode.pedroPathing.Pipelines.Sensor.*;
import static org.firstinspires.ftc.teamcode.pedroPathing.TeleOp.AprilTagCentererConfig.*;

@TeleOp(name = "AprilTag Centerer", group = "Testing")
public class AprilTag_Centerer extends LinearOpMode {

    private static final int TURRET_MIN_LIMIT = -45;
    private static final int TURRET_MAX_LIMIT = 840;
    private static final int WRAP_EXIT_MARGIN = 20;
    private static final double WRAP_SPEED = 0.6;

    @Override
    public void runOpMode() throws InterruptedException {
        intMotors(this);
        initLimelight(this);
        initSensors(this);

        turret.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        turret.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        ElapsedTime targetLostTimer = new ElapsedTime();

        double lastError = 0.0;
        double filteredError = 0.0;
        double lastSeenError = 0.0;
        double lastSeenTime = 0.0;
        double lastTxSign = 0.0;
        boolean overshootSlow = false;
        boolean scanDirectionRight = true;
        boolean wrapMode = false;
        boolean wrapDirectionRight = true;

        while (!isStarted() && !isStopRequested()) {
            if (isMagPressed()) {
                turret.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                turret.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            }
            int turretPosition = turret.getCurrentPosition();
            telemetry.addData("=== INIT: TURRET LIMITS ===", "");
            telemetry.addData("Turret Position", turretPosition);
            telemetry.addData("Min Limit", TURRET_MIN_LIMIT);
            telemetry.addData("Max Limit", TURRET_MAX_LIMIT);
            telemetry.addData("Mag Sensor", isMagPressed() ? "PRESSED (ZEROED)" : "NOT PRESSED");
            telemetry.addData("Tip", "Rotate turret and note min/max positions");
            telemetry.update();
            sleep(20);
        }

        waitForStart();
        double gear = 1.25; // speed modifier for drive train

        while (opModeIsActive()) {
            // Drive code (copied from Cassius)
            double LStickY = -gamepad1.right_stick_x;       // inverted
            double LStickX = -gamepad1.left_stick_x;
            double RStickX = gamepad1.left_stick_y; // inverted

            if (Math.abs(LStickX) > 0 || Math.abs(LStickY) > 0 || Math.abs(RStickX) > 0) {
                double r = Math.hypot(LStickX, LStickY);
                double robotAngle = Math.atan2(LStickY, LStickX) - Math.PI / 4;
                double rightX = RStickX;

                double v1 = r * Math.cos(robotAngle) + rightX * gear; // lf
                double v2 = r * Math.sin(robotAngle) - rightX * gear; // rf
                double v3 = r * Math.sin(robotAngle) + rightX * gear; // lb
                double v4 = r * Math.cos(robotAngle) - rightX * gear; // rb

                SetPower(v1, v3, v2, v4);
            } else {
                SetPower(0, 0, 0, 0);
            }

            if (isMagPressed()) {
                turret.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                turret.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            }

            double turretPower = 0.0;
            boolean hasTarget = hasBlueGoal();

            if (hasTarget) {
                double tx = getBlueGoalX();
                double txSign = Math.signum(tx);
                if (txSign != 0 && lastTxSign != 0 && txSign != lastTxSign) {
                    overshootSlow = true; // crossed center
                }
                if (txSign != 0) {
                    lastTxSign = txSign;
                }
                double rotationComp = LStickY * ROTATION_COMP;

                if (filteredError == 0.0 && lastError == 0.0) {
                    filteredError = tx;
                } else {
                    filteredError = FILTER_ALPHA * tx + (1.0 - FILTER_ALPHA) * filteredError;
                }

                if (Math.signum(tx) != 0) {
                    lastSeenError = tx;
                    lastSeenTime = getRuntime();
                }

                if (Math.abs(filteredError) > DEADBAND_DEG) {
                    turretPower = (filteredError * KP);
                    double scale = Math.tanh(Math.abs(filteredError));
                    turretPower *= scale;
                    if (overshootSlow && Math.signum(filteredError) != 0 && Math.signum(filteredError) == lastTxSign) {
                        turretPower *= 0.5; // slow down on return pass
                    }
                    lastError = filteredError;
                } else {
                    turretPower = 0.0;
                    overshootSlow = false;
                }

                turretPower += rotationComp;
                turretPower = Math.max(-MAX_TURRET_SPEED, Math.min(MAX_TURRET_SPEED, turretPower));

                targetLostTimer.reset();
            } else {
                // No target: slow scan to reacquire in last seen direction
                if (targetLostTimer.seconds() > 0.4) {
                    if (getRuntime() - lastSeenTime < 1.5) {
                        scanDirectionRight = lastSeenError < 0;
                    }
                    
                    turretPower = scanDirectionRight ? SCAN_POWER : -SCAN_POWER;
                    turretPower += 0.5 * LStickY * ROTATION_COMP;
                    turretPower = Math.max(-MAX_TURRET_SPEED, Math.min(MAX_TURRET_SPEED, turretPower));
                }
                overshootSlow = false;
            }

            int turretPosition = turret.getCurrentPosition();
            if (hasTarget && !wrapMode) {
                if (turretPosition <= TURRET_MIN_LIMIT && filteredError < 0) {
                    wrapMode = true;
                    wrapDirectionRight = true;
                }
                if (turretPosition >= TURRET_MAX_LIMIT && filteredError > 0) {
                    wrapMode = true;
                    wrapDirectionRight = false;
                }
            }

            if (wrapMode) {
                turretPower = wrapDirectionRight ? WRAP_SPEED : -WRAP_SPEED;
                if (hasTarget) {
                    if (wrapDirectionRight && filteredError > 0 && turretPosition > TURRET_MIN_LIMIT + WRAP_EXIT_MARGIN) {
                        wrapMode = false;
                    }
                    if (!wrapDirectionRight && filteredError < 0 && turretPosition < TURRET_MAX_LIMIT - WRAP_EXIT_MARGIN) {
                        wrapMode = false;
                    }
                }
            }
            if (turretPosition <= TURRET_MIN_LIMIT && turretPower < 0) {
                turretPower = 0.0;
            }
            if (turretPosition >= TURRET_MAX_LIMIT && turretPower > 0) {
                turretPower = 0.0;
            }

            turret.setPower(turretPower);

            telemetry.addData("=== APRILTAG CENTERER ===", "");
            telemetry.addData("Target", hasTarget ? "BLUE" : "NO");
            telemetry.addData("TX", hasTarget ? String.format("%.2f°", getBlueGoalX()) : "--");
            telemetry.addData("Filtered", String.format("%.2f°", filteredError));
            telemetry.addData("KP", String.format("%.4f", KP));
            telemetry.addData("Alpha", String.format("%.2f", FILTER_ALPHA));
            telemetry.addData("Turret Power", String.format("%.2f", turretPower));
            telemetry.addData("Rot Input", String.format("%.2f", RStickX));
            telemetry.addData("Rot Comp", String.format("%.2f", RStickX * ROTATION_COMP));
            telemetry.addData("Turret Pos", turretPosition);
            telemetry.update();
        }
    }
}
