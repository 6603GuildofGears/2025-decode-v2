package org.firstinspires.ftc.teamcode.pedroPathing.TeleOp.Test_codes;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;

import static org.firstinspires.ftc.teamcode.pedroPathing.Pipelines.Motor_PipeLine.*;
import static org.firstinspires.ftc.teamcode.pedroPathing.Pipelines.Servo_Pipeline.*;
import static org.firstinspires.ftc.teamcode.pedroPathing.Pipelines.Limelight_Pipeline.*;
import static org.firstinspires.ftc.teamcode.pedroPathing.TeleOp.HoodTestConfig.*;
import org.firstinspires.ftc.teamcode.pedroPathing.TeleOp.ShooterLookup;


@TeleOp(name="Flywheel & Hood Tuner", group="Testing")
public class Flywheel_Hood_Tuner extends LinearOpMode {

    // All values controlled ONLY by HoodTestConfig via Pedro Pathing Panels
    private double flickRest1  = 0.1;
    private double flickShoot1 = 0.55;

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

    // Flicker state machine
    private ElapsedTime flickTimer = new ElapsedTime();
    private ElapsedTime loopTimer = new ElapsedTime();
    private enum FlickState { REST, SWEEPING_TO_SHOOT, HOLDING, SWEEPING_TO_REST }
    private FlickState flickState = FlickState.REST;
    private boolean lastA = false;
    private double flickPos1; // current interpolated position

    
    @Override
    public void runOpMode() throws InterruptedException {
        
        intMotors(this);
        intServos(this);
        initLimelight(this);
        
        // Set flywheel to velocity mode for RPM control
        flywheel.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        
        telemetry.addData("Status", "Initialized");
        telemetry.addData("=== CONTROLS ===", "");
        telemetry.addData("ALL ADJUSTMENTS", "Use Pedro Pathing Panels");
        telemetry.addData("", "Open Panels to tune:");
        telemetry.addData("- HOOD_POSITION", "0.0 to 1.0");
        telemetry.addData("- TARGET_RPM", "0 to 6000");
        telemetry.addData("- HOOD_INCREMENT", "Adjustment step");
        telemetry.addData("- RPM_INCREMENT", "Adjustment step");
        telemetry.update();
        
        waitForStart();
        
        flickPos1 = flickRest1;
        loopTimer.reset();
        while (opModeIsActive()) {
            
            // === Limelight distance-based shooter tuning (same as Cassius_Blue) ===
            LLResult limelightResult = getLatestResult();
            double distanceInches = 0.0;
            boolean hasDistance = false;
            int detectedTagId = 0;
            double ty = 0.0;
            double tx = 0.0;

            if (limelightResult != null && limelightResult.isValid() &&
                limelightResult.getFiducialResults() != null && !limelightResult.getFiducialResults().isEmpty()) {
                // Find blue goal tag (ID 20) specifically — same as Cassius_Blue
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
                    double totalAngle = cameraMountAngle + ty;
                    double heightDifference = targetHeight - cameraHeight;
                    if (Math.abs(totalAngle) > 0.5 && Math.abs(totalAngle) < 89.5) {
                        distanceInches = heightDifference / Math.tan(Math.toRadians(totalAngle));
                        hasDistance = true;
                    }
                }
            }

            // Use ShooterLookup when distance is available (same logic as Cassius_Blue)
            if (hasDistance) {
                // Smooth distance to eliminate frame-to-frame jitter
                if (smoothedDistance < 0) {
                    smoothedDistance = distanceInches; // first reading — use raw
                } else {
                    smoothedDistance = smoothedDistance + SMOOTHING_ALPHA * (distanceInches - smoothedDistance);
                }
                // Lookup table is primary — always applies when target is visible
                ShooterLookup.Result tuned = ShooterLookup.lookup(smoothedDistance);
                targetRpm = tuned.rpm;
                targetHoodPos = tuned.hoodPos;
                hood.setPosition(targetHoodPos);
            } else {
                // No target — fall back to manual Panels values
                targetRpm = TARGET_RPM;
                targetHoodPos = HOOD_POSITION;
                hood.setPosition(targetHoodPos);
            }

            // Apply flywheel RPM
            ((DcMotorEx) flywheel).setVelocity(targetRpm * 28 / 60.0); // Convert RPM to ticks/sec

            // Get current flywheel RPM
            intake.setPower(0.5);
            double currentVelocity = ((DcMotorEx) flywheel).getVelocity(); // ticks per second
            double currentRPM = currentVelocity * 60.0 / 28.0; // Convert to RPM (28 ticks per revolution)

            // Flicker state machine with variable speed
            double dt = loopTimer.seconds();
            loopTimer.reset();
            double step = FLICK_SPEED * dt; // position change this loop

            boolean currentA = gamepad1.a;
            if (currentA && !lastA && flickState == FlickState.REST) {
                flickState = FlickState.SWEEPING_TO_SHOOT;
            }
            lastA = currentA;

            switch (flickState) {
                case REST:
                    flickPos1 = flickRest1;
                    break;
                case SWEEPING_TO_SHOOT:
                    flickPos1 = moveToward(flickPos1, flickShoot1, step);
                    if (flickPos1 == flickShoot1) {
                        flickState = FlickState.HOLDING;
                        flickTimer.reset();
                    }
                    break;
                case HOLDING:
                    if (flickTimer.seconds() >= FLICK_HOLD_TIME) {
                        flickState = FlickState.SWEEPING_TO_REST;
                    }
                    break;
                case SWEEPING_TO_REST:
                    flickPos1 = moveToward(flickPos1, flickRest1, step);
                    if (flickPos1 == flickRest1) {
                        flickState = FlickState.REST;
                    }
                    break;
            }
            flicker.setPosition(flickPos1);
            
            // Display telemetry
            telemetry.addData("=== LIMELIGHT ===", "");
            if (hasDistance) {
                telemetry.addData("Target Detected", "ID: %d", detectedTagId);
                telemetry.addData("Raw Distance", "%.2f in", distanceInches);
                telemetry.addData("Smoothed Distance", "%.2f in", smoothedDistance);
                telemetry.addData("TY Angle", "%.2f°", ty);
                telemetry.addData("Total Angle", "%.2f°", cameraMountAngle + ty);
                telemetry.addData("Source", "LOOKUP TABLE (auto)");
            } else {
                telemetry.addData("Target Detected", "None");
                telemetry.addData("Source", "PANELS (manual fallback)");
            }
            telemetry.addData("", "");
            
            telemetry.addData("=== HOOD ===", "");
            telemetry.addData("Hood Position", "%.3f", targetHoodPos);
            telemetry.addData("", "");
            
            telemetry.addData("=== FLYWHEEL ===", "");
            telemetry.addData("Target RPM", "%.0f", targetRpm);
            telemetry.addData("Current RPM", "%.0f", currentRPM);
            telemetry.addData("Error", "%.0f RPM", targetRpm - currentRPM);
            telemetry.addData("", "");
            telemetry.addData("=== FLICKER ===", "");
            telemetry.addData("Flick Speed", "%.1f pos/sec", FLICK_SPEED);
            telemetry.addData("Hold Time", "%.2f sec", FLICK_HOLD_TIME);
            telemetry.addData("Flick State", flickState.toString());
            telemetry.update();
        }
    }

    /** Move current toward target by at most step, clamped to target */
    private double moveToward(double current, double target, double step) {
        if (Math.abs(target - current) <= step) return target;
        return current + Math.signum(target - current) * step;
    }
}
