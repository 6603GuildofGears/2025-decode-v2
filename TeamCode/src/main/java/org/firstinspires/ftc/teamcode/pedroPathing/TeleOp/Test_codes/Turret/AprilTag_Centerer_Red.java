package org.firstinspires.ftc.teamcode.pedroPathing.TeleOp.Test_codes.Turret;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import static org.firstinspires.ftc.teamcode.pedroPathing.Pipelines.Motor_PipeLine.*;
import static org.firstinspires.ftc.teamcode.pedroPathing.Pipelines.Limelight_Pipeline.*;
import static org.firstinspires.ftc.teamcode.pedroPathing.Pipelines.Servo_Pipeline.*;
import static org.firstinspires.ftc.teamcode.pedroPathing.Pipelines.Sensor.*;
import static org.firstinspires.ftc.teamcode.pedroPathing.TeleOp.AprilTagCentererConfig.*;
import org.firstinspires.ftc.teamcode.pedroPathing.TeleOp.ShooterLookup;

@Disabled
@TeleOp(name = "AprilTag Centerer Red", group = "Testing")
public class AprilTag_Centerer_Red extends LinearOpMode {

    private static final int TURRET_MIN_LIMIT = -45;
    private static final int TURRET_MAX_LIMIT = 860;
    private static final int WRAP_EXIT_MARGIN = 20;
    private static final int LIMIT_SLOW_ZONE = 30;

    // Camera constants for distance calculation
    private static final double CAMERA_HEIGHT = 11.125; // inches
    private static final double CAMERA_MOUNT_ANGLE = 22.85; // degrees from horizontal (calibrated)
    private static final double TARGET_HEIGHT = 29.5; // AprilTag center height in inches

    @Override
    public void runOpMode() throws InterruptedException {
        intMotors(this);
        initLimelight(this);
        initSensors(this);
        intServos(this);

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
            int turretPosition = turret.getCurrentPosition();
            if (isMagPressed() && turretPosition > -25 && turretPosition < 25) {
                turret.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                turret.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            }
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
            // Drive code - simple mecanum
                double drive  = gamepad1.left_stick_y;
                double strafe = -gamepad1.left_stick_x;
                double turn   = -gamepad1.right_stick_x;

            if (Math.abs(drive) > 0.05 || Math.abs(strafe) > 0.05 || Math.abs(turn) > 0.05) {
                double leftFrontPower  = (drive + strafe + turn) * gear;
                double leftBackPower   = (drive - strafe + turn) * gear;
                double rightFrontPower = (drive - strafe - turn) * gear;
                double rightBackPower  = (drive + strafe - turn) * gear;

                double max = Math.max(Math.abs(leftFrontPower),
                             Math.max(Math.abs(leftBackPower),
                             Math.max(Math.abs(rightFrontPower),
                                      Math.abs(rightBackPower))));
                if (max > 1.0) {
                    leftFrontPower  /= max;
                    leftBackPower   /= max;
                    rightFrontPower /= max;
                    rightBackPower  /= max;
                }

                frontLeft.setPower(leftFrontPower);
                backLeft.setPower(leftBackPower);
                frontRight.setPower(rightFrontPower);
                backRight.setPower(rightBackPower);
            } else {
                SetPower(0, 0, 0, 0);
            }

            int turretPosition = turret.getCurrentPosition();
            // Runtime zeroing disabled to prevent position jumps
            // if (isMagPressed() && turretPosition > -25 && turretPosition < 25) {
            //     turret.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            //     turret.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            // }

            double turretPower = 0.0;
            boolean hasTarget = hasRedGoal();
            double rotationComp = drive * ROTATION_COMP;

            if (wrapMode) {
                // Whiparound takes precedence over search/tracking
                turretPower = wrapDirectionRight ? WRAP_SPEED : -WRAP_SPEED;
                turretPower += rotationComp;
                if (wrapDirectionRight && turretPower < 0) {
                    turretPower = WRAP_SPEED;
                }
                if (!wrapDirectionRight && turretPower > 0) {
                    turretPower = -WRAP_SPEED;
                }
                turretPower = Math.max(-MAX_TURRET_SPEED, Math.min(MAX_TURRET_SPEED, turretPower));
                if (hasTarget) {
                    wrapMode = false;
                    filteredError = 0.0;
                    lastError = 0.0;
                }
                // Min→Max whip stops 300 ticks early (field geometry)
                if (wrapDirectionRight && turretPosition >= TURRET_MAX_LIMIT - 300) {
                    wrapMode = false;
                    filteredError = 0.0;
                    lastError = 0.0;
                }
                if (!wrapDirectionRight && turretPosition <= TURRET_MIN_LIMIT + WRAP_EXIT_MARGIN) {
                    wrapMode = false;
                    filteredError = 0.0;
                    lastError = 0.0;
                }
            } else if (hasTarget) {
                double tx = getRedGoalX();
                double txSign = Math.signum(tx);
                if (txSign != 0 && lastTxSign != 0 && txSign != lastTxSign) {
                    overshootSlow = true; // crossed center
                }
                if (txSign != 0) {
                    lastTxSign = txSign;
                }
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

                if (Math.abs(filteredError) <= DEADBAND_DEG) {
                    rotationComp *= 0.5;
                }
                turretPower += rotationComp;
                turretPower = Math.max(-MAX_TURRET_SPEED, Math.min(MAX_TURRET_SPEED, turretPower));

                targetLostTimer.reset();

                // Distance-based hood adjustment
                double ty = getRedGoalY();
                double totalAngle = CAMERA_MOUNT_ANGLE + ty;
                double heightDiff = TARGET_HEIGHT - CAMERA_HEIGHT;
                if (Math.abs(totalAngle) > 0.5 && Math.abs(totalAngle) < 89.5) {
                    double distanceInches = heightDiff / Math.tan(Math.toRadians(totalAngle));
                    ShooterLookup.Result tuned = ShooterLookup.lookup(distanceInches);
                    hood.setPosition(tuned.hoodPos);
                }
            } else {
                // No target: apply rotation comp to hold position while waiting
                turretPower = rotationComp;
                turretPower = Math.max(-MAX_TURRET_SPEED, Math.min(MAX_TURRET_SPEED, turretPower));

                // After brief delay, start scanning to reacquire
                if (targetLostTimer.seconds() > 0.1) {
                    if (getRuntime() - lastSeenTime < 1.5) {
                        scanDirectionRight = lastSeenError < 0;
                    }
                    
                    turretPower = scanDirectionRight ? SCAN_POWER : -SCAN_POWER;
                    turretPower += 0.5 * rotationComp;
                    turretPower = Math.max(-MAX_TURRET_SPEED, Math.min(MAX_TURRET_SPEED, turretPower));

                    // Don't search past the 300-tick cap on the right side
                    if (scanDirectionRight && turretPosition >= TURRET_MAX_LIMIT - 300) {
                        scanDirectionRight = false;
                    }

                    if(turretPosition >= TURRET_MAX_LIMIT - LIMIT_SLOW_ZONE) {
                        turretPower /= 3.0; // slow down near limits
                    }
                    if(turretPosition <= TURRET_MIN_LIMIT + LIMIT_SLOW_ZONE) {
                        turretPower /= 3.0;
                    }
                }
                overshootSlow = false;
            }

            turretPosition = turret.getCurrentPosition();
            if (!wrapMode) {
                // Whip if turretPower wants to go past a limit
                // (covers tracking error, rotation comp, or both)
                if (turretPosition <= TURRET_MIN_LIMIT && turretPower < 0) {
                    wrapMode = true;
                    wrapDirectionRight = true;
                }
                if (turretPosition >= TURRET_MAX_LIMIT && turretPower > 0) {
                    wrapMode = true;
                    wrapDirectionRight = false;
                }
            }
            // Limit authority (hard stop + slow zone)
            if (turretPosition >= TURRET_MAX_LIMIT) {
                if (turretPower > 0) {
                    turretPower = 0.0;
                }
            } else if (turretPower > 0 && turretPosition > TURRET_MAX_LIMIT - LIMIT_SLOW_ZONE && (wrapMode || !hasTarget)) {
                double scale = (double) (TURRET_MAX_LIMIT - turretPosition) / LIMIT_SLOW_ZONE;
                turretPower *= Math.max(0.0, Math.min(1.0, scale));
            }

            if (turretPosition <= TURRET_MIN_LIMIT) {
                if (turretPower < 0) {
                    turretPower = 0.0;
                }
            } else if (turretPower < 0 && turretPosition < TURRET_MIN_LIMIT + LIMIT_SLOW_ZONE && (wrapMode || !hasTarget)) {
                double scale = (double) (turretPosition - TURRET_MIN_LIMIT) / LIMIT_SLOW_ZONE;
                turretPower *= Math.max(0.0, Math.min(1.0, scale));
            }

            turret.setPower(turretPower);

            telemetry.addData("=== APRILTAG CENTERER RED ===", "");
            telemetry.addData("Mode", wrapMode ? "WHIP " + (wrapDirectionRight ? "→" : "←") : (hasTarget ? "TRACK" : "SCAN"));
            telemetry.addData("Target", hasTarget ? "RED" : "NO");
            telemetry.addData("TX", hasTarget ? String.format("%.2f°", getRedGoalX()) : "--");
            telemetry.addData("Filtered", String.format("%.2f°", filteredError));
            telemetry.addData("KP", String.format("%.4f", KP));
            telemetry.addData("Alpha", String.format("%.2f", FILTER_ALPHA));
            telemetry.addData("Turret Power", String.format("%.2f", turretPower));
            telemetry.addData("Rot Input", String.format("%.2f", turn));
            telemetry.addData("Rot Comp", String.format("%.2f", turn * ROTATION_COMP));
            telemetry.addData("Turret Pos", turretPosition);
            if (hasTarget) {
                double tyDbg = getRedGoalY();
                double totalAngleDbg = CAMERA_MOUNT_ANGLE + tyDbg;
                double heightDiffDbg = TARGET_HEIGHT - CAMERA_HEIGHT;
                double distDbg = (Math.abs(totalAngleDbg) > 0.5 && Math.abs(totalAngleDbg) < 89.5)
                        ? heightDiffDbg / Math.tan(Math.toRadians(totalAngleDbg)) : 0.0;
                telemetry.addData("TY", String.format("%.2f°", tyDbg));
                telemetry.addData("Total Angle", String.format("%.2f°", totalAngleDbg));
                telemetry.addData("Distance", String.format("%.1f in", distDbg));
                telemetry.addData("Hood Pos", String.format("%.3f", hood.getPosition()));
            }
            telemetry.update();
        }
    }
}
