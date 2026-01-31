// package org.firstinspires.ftc.teamcode.pedroPathing.TeleOp.Test_codes;

// import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
// import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
// import com.qualcomm.robotcore.hardware.DcMotor;
// import com.qualcomm.robotcore.hardware.DcMotorEx;
// import com.qualcomm.robotcore.util.ElapsedTime;
// import com.qualcomm.hardware.limelightvision.LLResult;

// import static org.firstinspires.ftc.teamcode.pedroPathing.Pipelines.Motor_PipeLine.*;
// import static org.firstinspires.ftc.teamcode.pedroPathing.Pipelines.Servo_Pipeline.*;
// import static org.firstinspires.ftc.teamcode.pedroPathing.Pipelines.Limelight_Pipeline.*;

// @TeleOp(name="Flywheel & Hood Tuner", group="Testing")
// public class Flywheel_Hood_Tuner extends LinearOpMode {

//     private double hoodPosition = 0.0;  // Start at 0
//     private double flywheelPower = 0.0; // Start at 0
//     private int targetRPM = 0;
    
//     private ElapsedTime buttonTimer = new ElapsedTime();
//     private final double BUTTON_DELAY = 0.15; // Delay between button presses (seconds)
    
//     // Increment values
//     private final double HOOD_INCREMENT = 0.01;  // Small increment for hood
//     private final double POWER_INCREMENT = 0.05; // Power increment
//     private final int RPM_INCREMENT = 50;        // RPM increment
    
//     @Override
//     public void runOpMode() throws InterruptedException {
        
//         intMotors(this);
//         intServos(this);
//         initLimelight(this);
        
//         // Set flywheel to velocity mode for RPM control
//         flywheel.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        
//         telemetry.addData("Status", "Initialized");
//         telemetry.addData("Instructions", "Use D-Pad to adjust");
//         telemetry.addData("", "UP/DOWN = Hood Position");
//         telemetry.addData("", "LEFT/RIGHT = Flywheel Power");
//         telemetry.addData("", "Y/A = Flywheel RPM");
//         telemetry.addData("", "B = Reset All to 0");
//         telemetry.update();
        
//         waitForStart();
//         buttonTimer.reset();
        
//         while (opModeIsActive()) {
            
//             // Hood position control (D-pad Up/Down)
//             if (buttonTimer.seconds() > BUTTON_DELAY) {
//                 if (gamepad1.dpad_up) {
//                     hoodPosition += HOOD_INCREMENT;
//                     if (hoodPosition > 1.0) hoodPosition = 1.0;
//                     buttonTimer.reset();
//                 }
//                 if (gamepad1.dpad_down) {
//                     hoodPosition -= HOOD_INCREMENT;
//                     if (hoodPosition < 0.0) hoodPosition = 0.0;
//                     buttonTimer.reset();
//                 }
                
//                 // Flywheel power control (D-pad Left/Right)
//                 if (gamepad1.dpad_right) {
//                     flywheelPower += POWER_INCREMENT;
//                     if (flywheelPower > 1.0) flywheelPower = 1.0;
//                     targetRPM = 0; // Reset RPM when using power control
//                     buttonTimer.reset();
//                 }
//                 if (gamepad1.dpad_left) {
//                     flywheelPower -= POWER_INCREMENT;
//                     if (flywheelPower < 0.0) flywheelPower = 0.0;
//                     targetRPM = 0;
//                     buttonTimer.reset();
//                 }
                
//                 // Flywheel RPM control (Y/A buttons)
//                 if (gamepad1.y) {
//                     targetRPM += RPM_INCREMENT;
//                     if (targetRPM > 6000) targetRPM = 6000; // Safety limit
//                     flywheelPower = 0.0; // Reset power when using RPM control
//                     buttonTimer.reset();
//                 }
//                 if (gamepad1.a) {
//                     targetRPM -= RPM_INCREMENT;
//                     if (targetRPM < 0) targetRPM = 0;
//                     flywheelPower = 0.0;
//                     buttonTimer.reset();
//                 }
                
//                 // Reset all (B button)
//                 if (gamepad1.b) {
//                     hoodPosition = 0.0;
//                     flywheelPower = 0.0;
//                     targetRPM = 0;
//                     buttonTimer.reset();
//                 }
//             }
            
//             // Apply hood position
//             hood.setPosition(hoodPosition);
            
//             // Apply flywheel control (either power or RPM)
//             if (targetRPM > 0) {
//                 // Use velocity (RPM) control
//                 ((DcMotorEx) flywheel).setVelocity(targetRPM * 28 / 60.0); // Convert RPM to ticks/sec
//             } else {
//                 // Use power control
//                 flywheel.setPower(flywheelPower);
//             }
            
//             // Get current flywheel RPM
//             double currentVelocity = ((DcMotorEx) flywheel).getVelocity(); // ticks per second
//             double currentRPM = currentVelocity * 60.0 / 28.0; // Convert to RPM (assuming 28 ticks per revolution)
            
//             // Get limelight distance
//             LLResult limelightResult = getLatestResult();
//             double distance = 0.0;
//             int detectedTagId = 0;
            
//             if (limelightResult != null && limelightResult.isValid() && 
//                 limelightResult.getFiducialResults() != null && !limelightResult.getFiducialResults().isEmpty()) {
//                 // Get the first detected tag
//                 detectedTagId = (int) limelightResult.getFiducialResults().get(0).getFiducialId();
                
//                 // Calculate distance using ty (vertical offset) - typical calculation for AprilTag
//                 // You may need to adjust this formula based on your camera mounting
//                 double ty = limelightResult.getFiducialResults().get(0).getTargetYDegrees();
                
//                 // Simple distance estimation (adjust constants for your setup)
//                 // distance = height_difference / tan(angle)
//                 distance = 20.0 / Math.tan(Math.toRadians(ty)); // Rough estimation, adjust as needed
//             }
            
//             // Display telemetry
//             telemetry.addData("=== LIMELIGHT ===", "");
//             if (detectedTagId > 0) {
//                 telemetry.addData("Target Detected", "ID: %d", detectedTagId);
//                 telemetry.addData("Distance", "%.2f inches", distance);
//             } else {
//                 telemetry.addData("Target Detected", "None");
//             }
//             telemetry.addData("", "");
            
//             telemetry.addData("=== HOOD ===", "");
//             telemetry.addData("Hood Position", "%.3f", hoodPosition);
//             telemetry.addData("", "");
            
//             telemetry.addData("=== FLYWHEEL ===", "");
//             if (targetRPM > 0) {
//                 telemetry.addData("Control Mode", "RPM");
//                 telemetry.addData("Target RPM", targetRPM);
//                 telemetry.addData("Current RPM", "%.0f", currentRPM);
//             } else {
//                 telemetry.addData("Control Mode", "Power");
//                 telemetry.addData("Power", "%.2f", flywheelPower);
//                 telemetry.addData("Current RPM", "%.0f", currentRPM);
//             }
//             telemetry.addData("", "");
            
//             telemetry.addData("=== CONTROLS ===", "");
//             telemetry.addData("D-Pad Up/Down", "Hood ±%.3f", HOOD_INCREMENT);
//             telemetry.addData("D-Pad Left/Right", "Power ±%.2f", POWER_INCREMENT);
//             telemetry.addData("Y/A", "RPM ±%d", RPM_INCREMENT);
//             telemetry.addData("B", "Reset All");
//             telemetry.update();
//         }
//     }
// }
