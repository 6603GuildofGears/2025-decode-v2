package org.firstinspires.ftc.teamcode.pedroPathing.TeleOp.Test_codes;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.hardware.limelightvision.LLResult;

import static org.firstinspires.ftc.teamcode.pedroPathing.Pipelines.Motor_PipeLine.*;
import static org.firstinspires.ftc.teamcode.pedroPathing.Pipelines.Servo_Pipeline.*;
import static org.firstinspires.ftc.teamcode.pedroPathing.Pipelines.Limelight_Pipeline.*;
import static org.firstinspires.ftc.teamcode.pedroPathing.TeleOp.HoodTestConfig.*;

@Disabled
@TeleOp(name="Flywheel & Hood Tuner", group="Testing")
public class Flywheel_Hood_Tuner extends LinearOpMode {

    // All values controlled ONLY by HoodTestConfig via Pedro Pathing Panels
    private double flickRest1  = 0.1;
    private double flickRest2  = 0.1;
    private double flickShoot1 = 0.55;
    private double flickShoot2 = 0.5;

    // Flicker state machine
    private ElapsedTime flickTimer = new ElapsedTime();
    private ElapsedTime loopTimer = new ElapsedTime();
    private enum FlickState { REST, SWEEPING_TO_SHOOT, HOLDING, SWEEPING_TO_REST }
    private FlickState flickState = FlickState.REST;
    private boolean lastA = false;
    private double flickPos1, flickPos2; // current interpolated positions

    
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
        flickPos2 = flickRest2;
        loopTimer.reset();
        while (opModeIsActive()) {
            
            // All values read directly from HoodTestConfig (updated live via Panels)
            
            // Apply hood position from config
            hood.setPosition(HOOD_POSITION);
            
            // Apply flywheel RPM control using config value
            ((DcMotorEx) flywheel).setVelocity(TARGET_RPM * 28 / 60.0); // Convert RPM to ticks/sec
            
            // Get current flywheel RPM
            intake.setPower(0.5);
            double currentVelocity = ((DcMotorEx) flywheel).getVelocity(); // ticks per second
            double currentRPM = currentVelocity * 60.0 / 28.0; // Convert to RPM (assuming 28 ticks per revolution)
            
            // Get limelight distance
            LLResult limelightResult = getLatestResult();
            double distance = 0.0;
            int detectedTagId = 0;
            double ty = 0.0;
            double tx = 0.0;
            
            // Camera configuration
            double cameraHeight = 11.125; // Camera lens center height in inches
            double cameraMountAngle = 22.85; // Camera angle from horizontal (calibrated)
            double targetHeight = 29.5; // AprilTag center height in inches
            
            if (limelightResult != null && limelightResult.isValid() && 
                limelightResult.getFiducialResults() != null && !limelightResult.getFiducialResults().isEmpty()) {
                // Get the first detected tag
                detectedTagId = (int) limelightResult.getFiducialResults().get(0).getFiducialId();
                ty = limelightResult.getFiducialResults().get(0).getTargetYDegrees();
                tx = limelightResult.getFiducialResults().get(0).getTargetXDegrees();
                
                // Distance calculation with angled camera
                // For upward-angled camera, ADD ty (ty is negative when target is below crosshair)
                double totalAngle = cameraMountAngle + ty;
                double heightDifference = targetHeight - cameraHeight;
                
                if (Math.abs(totalAngle) > 0.5 && Math.abs(totalAngle) < 89.5) { // Avoid extreme angles
                    distance = heightDifference / Math.tan(Math.toRadians(totalAngle));
                }
            }

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
                    flickPos2 = flickRest2;
                    break;
                case SWEEPING_TO_SHOOT:
                    flickPos1 = moveToward(flickPos1, flickShoot1, step);
                    flickPos2 = moveToward(flickPos2, flickShoot2, step);
                    if (flickPos1 == flickShoot1 && flickPos2 == flickShoot2) {
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
                    flickPos2 = moveToward(flickPos2, flickRest2, step);
                    if (flickPos1 == flickRest1 && flickPos2 == flickRest2) {
                        flickState = FlickState.REST;
                    }
                    break;
            }
            flicker1.setPosition(flickPos1);
            flicker2.setPosition(flickPos2);
            
            // Display telemetry
            telemetry.addData("=== LIMELIGHT ===", "");
            if (detectedTagId > 0) {
                telemetry.addData("Target Detected", "ID: %d", detectedTagId);
                telemetry.addData("Distance", "%.2f inches", distance);
                telemetry.addData("TY Angle", "%.2f°", ty);
                telemetry.addData("Total Angle", "%.2f°", cameraMountAngle + ty);
            } else {
                telemetry.addData("Target Detected", "None");
            }
            telemetry.addData("", "");
            
            telemetry.addData("=== HOOD ===", "");
            telemetry.addData("Hood Position", "%.3f", HOOD_POSITION);
            telemetry.addData("", "");
            
            telemetry.addData("=== FLYWHEEL ===", "");
            telemetry.addData("Target RPM", "%.0f", TARGET_RPM);
            telemetry.addData("Current RPM", "%.0f", currentRPM);
            telemetry.addData("Error", "%.0f RPM", TARGET_RPM - currentRPM);
            telemetry.addData("", "");
            
            telemetry.addData("=== ADJUSTMENTS ===", "");
            telemetry.addData("Control Method", "Pedro Pathing Panels ONLY");
            telemetry.addData("Hood Increment", "±%.3f", HOOD_INCREMENT);
            telemetry.addData("RPM Increment", "±%.0f", RPM_INCREMENT);
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
