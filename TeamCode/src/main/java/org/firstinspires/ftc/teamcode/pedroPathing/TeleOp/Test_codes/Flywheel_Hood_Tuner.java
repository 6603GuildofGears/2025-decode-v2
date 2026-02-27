package org.firstinspires.ftc.teamcode.pedroPathing.TeleOp.Test_codes;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;

import static org.firstinspires.ftc.teamcode.pedroPathing.Pipelines.Motor_PipeLine.*;
import static org.firstinspires.ftc.teamcode.pedroPathing.Pipelines.Servo_Pipeline.*;
import static org.firstinspires.ftc.teamcode.pedroPathing.Pipelines.Limelight_Pipeline.*;
import static org.firstinspires.ftc.teamcode.pedroPathing.Pipelines.Sensor.*;
import static org.firstinspires.ftc.teamcode.pedroPathing.TeleOp.HoodTestConfig.*;
import org.firstinspires.ftc.teamcode.pedroPathing.TeleOp.ShooterLookup;
import org.firstinspires.ftc.teamcode.pedroPathing.Pipelines.SpindexerController;

@Disabled
@TeleOp(name="Flywheel & Hood Tuner", group="Testing")
public class Flywheel_Hood_Tuner extends LinearOpMode {

    // Limelight distance config (same as Cassius_Blue)
    private double cameraHeight = 11.125;      // Camera lens center height in inches
    private double cameraMountAngle = 22.85;   // Camera angle from horizontal (calibrated)
    private double targetHeight = 29.5;        // AprilTag center height in inches

    // Smoothing filter for distance — prevents hood jitter from noisy Limelight frames
    private double smoothedDistance = -1;      // -1 means no value yet
    private double SMOOTHING_ALPHA = 0.3;      // 0.0 = very smooth/slow, 1.0 = no smoothing (raw)

    // Shooter state
    private double baseRpm = 3000;
    private double targetRpm = 3000;
    private double targetHoodPos = 0.0;

    @Override
    public void runOpMode() throws InterruptedException {

        intMotors(this);
        intServos(this);
        initLimelight(this);
        initSensors(this);

        // Set flywheel to velocity mode for RPM control
        flywheel.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        // SpindexerController — same as Cassius_Blue
        // Handles: intake ball detection, slot tracking, 3-slot shoot sequence,
        //          RPM boosting, flicker timing, misfire retry
        SpindexerController sdx = new SpindexerController();
        sdx.setFlickerPositions(0, 0.375);
        flicker.setPosition(0);

        telemetry.addData("Status", "Initialized");
        telemetry.addData("=== CONTROLS ===", "");
        telemetry.addData("RT (gp1)", "Hold to intake balls");
        telemetry.addData("RBumper (gp2)", "Shoot all 3 slots");
        telemetry.addData("LBumper (gp2)", "Kill switch (abort shoot)");
        telemetry.addData("RBumper (gp1)", "Reverse intake");
        telemetry.addData("DpadUp/Down (gp2)", "Manual hood adjust (no target)");
        telemetry.update();

        // Init loop — show spindexer status while waiting for start
        while (!isStarted() && !isStopRequested()) {
            updateSensors();
            sdx.addTelemetry(telemetry);
            telemetry.update();
        }

        if (isStopRequested()) return;
        spindexerAxon.setTargetRotation(SpindexerController.P1); // start at P1

        while (opModeIsActive()) {

            // Update sensors every loop
            updateSensors();

            // === Gamepad inputs ===
            double RTrigger1 = gamepad1.right_trigger;
            boolean RBumper1 = gamepad1.right_bumper;
            boolean RBumper2 = gamepad2.right_bumper;
            boolean LBumper2 = gamepad2.left_bumper;
            boolean dpadUp2 = gamepad2.dpad_up;
            boolean dpadDown2 = gamepad2.dpad_down;

            // === Intake with SpindexerController (same as Cassius_Blue) ===
            boolean intakeRequested = RTrigger1 > 0.1;
            boolean runIntakeMotor = sdx.updateIntake(intakeRequested);

            if (runIntakeMotor) {
                intake.setPower(0.875);
                intake2.setPower(1);
            } else if (sdx.isShooting()) {
                intake.setPower(0);
                intake2.setPower(0);
            } else if (RBumper1) {
                intake.setPower(-0.875); // reverse intake
                intake2.setPower(-0.875);
            } else {
                intake.setPower(0);
                intake2.setPower(0);
            }

            // === Limelight distance-based shooter tuning ===
            LLResult limelightResult = getLatestResult();
            double distanceInches = 0.0;
            double trigDistanceInches = 0.0;
            boolean hasDistance = false;
            int detectedTagId = 0;
            double ty = 0.0;
            double tx = 0.0;

            if (limelightResult != null && limelightResult.isValid() &&
                limelightResult.getFiducialResults() != null && !limelightResult.getFiducialResults().isEmpty()) {
                // Find blue goal tag (ID 20) specifically
                LLResultTypes.FiducialResult blueGoalForHood = null;
                for (LLResultTypes.FiducialResult f : limelightResult.getFiducialResults()) {
                    detectedTagId = (int) f.getFiducialId();
                    if (detectedTagId == 20) {
                        blueGoalForHood = f;
                        break;
                    }
                }
                if (blueGoalForHood != null) {
                    ty = blueGoalForHood.getTargetYDegrees();
                    tx = blueGoalForHood.getTargetXDegrees();

                    // --- 3D pose distance (primary — more accurate) ---
                    Pose3D tagPose = blueGoalForHood.getTargetPoseCameraSpace();
                    if (tagPose != null) {
                        // Camera space: X = right, Y = down, Z = forward (meters)
                        double xMeters = tagPose.getPosition().x;
                        double zMeters = tagPose.getPosition().z;
                        // Horizontal distance on the ground plane (ignore vertical)
                        distanceInches = Math.sqrt(xMeters * xMeters + zMeters * zMeters) * 39.3701;
                        hasDistance = true;
                    }

                    // --- Trig distance (fallback + telemetry comparison) ---
                    double totalAngle = cameraMountAngle + ty;
                    double heightDifference = targetHeight - cameraHeight;
                    if (Math.abs(totalAngle) > 0.5 && Math.abs(totalAngle) < 89.5) {
                        trigDistanceInches = heightDifference / Math.tan(Math.toRadians(totalAngle));
                        if (!hasDistance) {
                            distanceInches = trigDistanceInches;
                            hasDistance = true;
                        }
                    }
                }
            }

            // Use ShooterLookup when distance is available (same logic as Cassius_Blue)
            if (hasDistance) {
                if (smoothedDistance < 0) {
                    smoothedDistance = distanceInches;
                } else {
                    smoothedDistance = smoothedDistance + SMOOTHING_ALPHA * (distanceInches - smoothedDistance);
                }
                ShooterLookup.Result tuned = ShooterLookup.lookup(smoothedDistance);
                targetRpm = tuned.rpm;
                targetHoodPos = tuned.hoodPos;
                hood.setPosition(targetHoodPos);
                sdx.setShootRpm(targetRpm);
            } else {
                // No target — fall back to manual Panels values
                targetRpm = TARGET_RPM;
                targetHoodPos = HOOD_POSITION;
                sdx.setShootRpm(targetRpm);
                // Manual hood adjust via dpad when no target
                double currentHoodPos = hood.getPosition();
                if (dpadUp2) {
                    hood.setPosition(Math.min(1.0, currentHoodPos + 0.01));
                } else if (dpadDown2) {
                    hood.setPosition(Math.max(0.0, currentHoodPos - 0.01));
                } else {
                    hood.setPosition(targetHoodPos);
                }
            }

            // Apply flywheel RPM only when NOT shooting — SpindexerController owns flywheel during shoot
            if (!sdx.isShooting()) {
                ((DcMotorEx) flywheel).setVelocity(targetRpm * 28 / 60.0);
            }

            // Get current flywheel RPM
            double currentVelocity = ((DcMotorEx) flywheel).getVelocity();
            double currentRPM = currentVelocity * 60.0 / 28.0;

            // === Shooting sequence (same as Cassius_Blue) ===
            // SpindexerController handles everything:
            //   slot tracking, skip empties, smart direction, flicker timing
            //   RBumper2 = shoot, LBumper2 = kill switch
            sdx.updateShoot(RBumper2, LBumper2, flywheel);

            // === Telemetry ===
            telemetry.addData("=== LIMELIGHT ===", "");
            if (hasDistance) {
                telemetry.addData("Target", "ID: %d", detectedTagId);
                telemetry.addData("3D Pose Distance", "%.2f in", distanceInches);
                telemetry.addData("Trig Distance", "%.2f in", trigDistanceInches);
                telemetry.addData("Smoothed Distance", "%.2f in", smoothedDistance);
                telemetry.addData("TY Angle", "%.2f°", ty);
                telemetry.addData("Source", "3D POSE (auto)");;
            } else {
                telemetry.addData("Target", "None");
                telemetry.addData("Source", "PANELS (manual fallback)");
            }
            telemetry.addData("", "");

            telemetry.addData("=== SHOOTER ===", "");
            telemetry.addData("Target RPM", "%.0f", targetRpm);
            telemetry.addData("Current RPM", "%.0f", currentRPM);
            telemetry.addData("Error", "%.0f RPM", targetRpm - currentRPM);
            telemetry.addData("Hood Position", "%.3f", hood.getPosition());
            telemetry.addData("", "");

            telemetry.addData("=== SPINDEXER ===", "");
            telemetry.addData("Ball Present", isBallPresent());
            telemetry.addData("Ball Color", isBallPresent() ? detectBallColor() : "--");
            telemetry.addData("Spindexer Angle", "%.1f°", spindexerAxon.getRawAngle());
            sdx.addTelemetry(telemetry);

            telemetry.update();
        }
    }
}

