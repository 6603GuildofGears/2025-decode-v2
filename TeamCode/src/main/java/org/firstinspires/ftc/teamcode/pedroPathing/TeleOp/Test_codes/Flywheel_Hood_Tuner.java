package org.firstinspires.ftc.teamcode.pedroPathing.TeleOp.Test_codes;

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

@TeleOp(name="Flywheel & Hood Tuner", group="Testing")
public class Flywheel_Hood_Tuner extends LinearOpMode {

    // Values now controlled by HoodTestConfig for live tuning via Pedro Pathing Panels
    
    private ElapsedTime buttonTimer = new ElapsedTime();
    private final double BUTTON_DELAY = 0.15; // Delay between button presses (seconds)
    
    @Override
    public void runOpMode() throws InterruptedException {
        
        intMotors(this);
        intServos(this);
        initLimelight(this);
        
        // Set flywheel to velocity mode for RPM control
        flywheel.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        
        telemetry.addData("Status", "Initialized");
        telemetry.addData("=== CONTROLS ===", "");
        telemetry.addData("D-Pad UP/DOWN", "Hood Position");
        telemetry.addData("Y/A Buttons", "Flywheel RPM");
        telemetry.addData("B Button", "Reset All to Config Values");
        telemetry.addData("", "");
        telemetry.addData("TIP:", "Use Pedro Pathing Panels for live tuning!");
        telemetry.update();
        
        waitForStart();
        buttonTimer.reset();
        
        while (opModeIsActive()) {
            
            // Hood position control (D-pad Up/Down) - modifies the config value
            if (buttonTimer.seconds() > BUTTON_DELAY) {
                if (gamepad1.dpad_up) {
                    HOOD_POSITION += HOOD_INCREMENT;
                    if (HOOD_POSITION > 1.0) HOOD_POSITION = 1.0;
                    buttonTimer.reset();
                }
                if (gamepad1.dpad_down) {
                    HOOD_POSITION -= HOOD_INCREMENT;
                    if (HOOD_POSITION < 0.0) HOOD_POSITION = 0.0;
                    buttonTimer.reset();
                }
                
                // Flywheel RPM control (Y/A buttons) - modifies the config value
                if (gamepad1.y) {
                    TARGET_RPM += RPM_INCREMENT;
                    if (TARGET_RPM > 6000) TARGET_RPM = 6000; // Safety limit
                    buttonTimer.reset();
                }
                if (gamepad1.a) {
                    TARGET_RPM -= RPM_INCREMENT;
                    if (TARGET_RPM < 0) TARGET_RPM = 0;
                    buttonTimer.reset();
                }
                
                // Reset all to config baseline values (B button)
                if (gamepad1.b) {
                    HOOD_POSITION = 0.0;
                    TARGET_RPM = 3000.0;
                    buttonTimer.reset();
                }
            }
            
            // Apply hood position from config
            hood.setPosition(HOOD_POSITION);
            
            // Apply flywheel RPM control using config value
            ((DcMotorEx) flywheel).setVelocity(TARGET_RPM * 28 / 60.0); // Convert RPM to ticks/sec
            
            // Get current flywheel RPM
            double currentVelocity = ((DcMotorEx) flywheel).getVelocity(); // ticks per second
            double currentRPM = currentVelocity * 60.0 / 28.0; // Convert to RPM (assuming 28 ticks per revolution)
            
            // Get limelight distance
            LLResult limelightResult = getLatestResult();
            double distance = 0.0;
            int detectedTagId = 0;
            
            if (limelightResult != null && limelightResult.isValid() && 
                limelightResult.getFiducialResults() != null && !limelightResult.getFiducialResults().isEmpty()) {
                // Get the first detected tag
                detectedTagId = (int) limelightResult.getFiducialResults().get(0).getFiducialId();
                
                // Calculate distance using ty (vertical offset) - typical calculation for AprilTag
                // You may need to adjust this formula based on your camera mounting
                double ty = limelightResult.getFiducialResults().get(0).getTargetYDegrees();
                
                // Simple distance estimation (adjust constants for your setup)
                // distance = height_difference / tan(angle)
                distance = 20.0 / Math.tan(Math.toRadians(ty)); // Rough estimation, adjust as needed
            }
            
            // Display telemetry
            telemetry.addData("=== LIMELIGHT ===", "");
            if (detectedTagId > 0) {
                telemetry.addData("Target Detected", "ID: %d", detectedTagId);
                telemetry.addData("Distance", "%.2f inches", distance);
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
            
            telemetry.addData("=== CONTROLS ===", "");
            telemetry.addData("D-Pad Up/Down", "Hood ±%.3f", HOOD_INCREMENT);
            telemetry.addData("Y/A", "RPM ±%.0f", RPM_INCREMENT);
            telemetry.addData("B", "Reset to Baseline");
            telemetry.addData("", "");
            telemetry.addData("TIP:", "Use Pedro Pathing Panels for live tuning!");
            telemetry.update();
        }
    }
}
