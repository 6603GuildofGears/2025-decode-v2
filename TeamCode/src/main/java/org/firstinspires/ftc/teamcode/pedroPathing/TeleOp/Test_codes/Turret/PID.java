// package org.firstinspires.ftc.teamcode.pedroPathing.TeleOp.Test_codes;

// import com.qualcomm.robotcore.eventloop.opmode.OpMode;
// import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
// import com.qualcomm.robotcore.hardware.DcMotor;
// import com.qualcomm.robotcore.hardware.PIDFCoefficients;

// import static org.firstinspires.ftc.teamcode.pedroPathing.Pipelines.Motor_PipeLine.*;
// import static org.firstinspires.ftc.teamcode.pedroPathing.Pipelines.Servo_Pipeline.*;

// @TeleOp(name="PID Test", group="Tests")
// public class PID extends OpMode {

//     // PID tuning variables
//     double kP = 10.0;
//     double kI = 0.0;
//     double kD = 0.0;
//     double kF = 0.0;
//     double targetRPM = 4000;
    
//     // Adjustment increments
//     double pIncrement = 0.5;
//     double iIncrement = 0.1;
//     double dIncrement = 0.1;
//     double fIncrement = 0.1;
//     double rpmIncrement = 100;
    
//     boolean flywheelActive = false;
//     boolean xPressed = false; // For debouncing X button

//     @Override
//     public void init() {
//         // Initialization code here
//         intMotors(this);
//         intServos(this);
        
//         // Reset turret encoder to 0 at current position
//         turret.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//         turret.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        
//         // Set flywheel to velocity control mode
//         flywheel.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        
//         telemetry.addData("Status", "Initialized - PID Tuning Ready");
//         telemetry.update();
//     }

//     @Override
//     public void loop() {
//         // Get gamepad inputs
//         double turretPower = -gamepad2.right_stick_x; // Right stick X for turret
//         boolean flickerShoot = gamepad2.a; // A button for flicker
//         boolean flicker2Shoot = gamepad2.b; // B button for flicker2
//         boolean spindexerForward = gamepad2.dpad_right; // Dpad right for spindexer forward
//         boolean spindexerReverse = gamepad2.dpad_left; // Dpad left for spindexer reverse
        
//         // PID tuning controls (Gamepad 1)
//         // P adjustment
//         if (gamepad1.dpad_up) {
//             kP += pIncrement;
//         } else if (gamepad1.dpad_down) {
//             kP = Math.max(0, kP - pIncrement);
//         }
        
//         // I adjustment
//         if (gamepad1.dpad_right) {
//             kI += iIncrement;
//         } else if (gamepad1.dpad_left) {
//             kI = Math.max(0, kI - iIncrement);
//         }
        
//         // D adjustment
//         if (gamepad1.right_bumper) {
//             kD += dIncrement;
//         } else if (gamepad1.left_bumper) {
//             kD = Math.max(0, kD - dIncrement);
//         }
        
//         // F adjustment
//         if (gamepad1.y) {
//             kF += fIncrement;
//         } else if (gamepad1.a) {
//             kF = Math.max(0, kF - fIncrement);
//         }
        
//         // Target RPM adjustment
//         if (gamepad1.right_trigger > 0.5) {
//             targetRPM += rpmIncrement;
//         } else if (gamepad1.left_trigger > 0.5) {
//             targetRPM = Math.max(0, targetRPM - rpmIncrement);
//         }
        
//         // Toggle flywheel on/off
//         if (gamepad1.x && !xPressed) {
//             flywheelActive = !flywheelActive;
//             xPressed = true;
//         } else if (!gamepad1.x) {
//             xPressed = false;
//         }
        
//         // Apply PID coefficients
//         PIDFCoefficients pidfCoefficients = new PIDFCoefficients(kP, kI, kD, kF);
//         flywheel.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidfCoefficients);
        
//         // Set flywheel velocity
//         if (flywheelActive) {
//             flywheel.setVelocity(getTickSpeed(targetRPM));
//         } else {
//             flywheel.setVelocity(0);
//         }
        
//         // Turret control with position tracking
//         turret.setPower(turretPower * 0.5); // Half power for precise control
        
//         // Flicker control
//         if (flickerShoot) {
//             flicker.setPosition(0.325); // Shoot position
//         } else {
//             flicker.setPosition(0.0); // Rest position
//         }
        
//         // Flicker2 control
//         if (flicker2Shoot) {
//             flicker2.setPosition(0.325); // Shoot position
//         } else {
//             flicker2.setPosition(0.0); // Rest position
//         }
        
//         // Spindexer control
//         if (spindexerForward) {
//             spindexer.setPower(0.5); // Forward at half power
//         } else if (spindexerReverse) {
//             spindexer.setPower(-0.5); // Reverse at half power
//         } else {
//             spindexer.setPower(0); // Stop
//         }
        
//         // Calculate current RPM
//         double currentVelocity = flywheel.getVelocity();
//         double currentRPM = (currentVelocity * 60.0) / 537.7; // Ticks per minute to RPM
        
//         // Telemetry
//         telemetry.addData("=== FLYWHEEL STATUS ===", "");
//         telemetry.addData("Status", flywheelActive ? "ACTIVE" : "OFF (Press X to start)");
//         telemetry.addData("Target RPM", "%.0f", targetRPM);
//         telemetry.addData("Current RPM", "%.0f", currentRPM);
//         telemetry.addData("Error", "%.0f RPM", targetRPM - currentRPM);
//         telemetry.addData("", "");
//         telemetry.addData("=== PID COEFFICIENTS ===", "");
//         telemetry.addData("kP", "%.2f (Dpad Up/Down)", kP);
//         telemetry.addData("kI", "%.2f (Dpad L/R)", kI);
//         telemetry.addData("kD", "%.2f (Bumpers)", kD);
//         telemetry.addData("kF", "%.2f (Y/A)", kF);
//         telemetry.addData("", "");
//         telemetry.addData("=== GAMEPAD 1 CONTROLS ===", "");
//         telemetry.addData("Dpad Up/Down", "Adjust kP (+/-" + pIncrement + ")");
//         telemetry.addData("Dpad Left/Right", "Adjust kI (+/-" + iIncrement + ")");
//         telemetry.addData("Bumpers", "Adjust kD (+/-" + dIncrement + ")");
//         telemetry.addData("Y/A Buttons", "Adjust kF (+/-" + fIncrement + ")");
//         telemetry.addData("Triggers", "Adjust RPM (+/-" + (int)rpmIncrement + ")");
//         telemetry.addData("X Button", "Toggle Flywheel On/Off");
//         telemetry.addData("", "");
//         telemetry.addData("=== GAMEPAD 2 CONTROLS ===", "");
//         telemetry.addData("Right Stick X", "Turret (Pos: " + turret.getCurrentPosition() + ")");
//         telemetry.addData("A Button", "Flicker (" + String.format("%.2f", flicker.getPosition()) + ")");
//         telemetry.addData("B Button", "Flicker2 (" + String.format("%.2f", flicker2.getPosition()) + ")");
//         telemetry.addData("Dpad L/R", "Spindexer (" + (spindexerForward ? "FWD" : (spindexerReverse ? "REV" : "OFF")) + ")");
//         telemetry.addData("", "");
//         telemetry.addData("=== TUNING TIPS ===", "");
//         telemetry.addData("1", "Start with kP, increase until oscillation");
//         telemetry.addData("2", "Add kD to reduce oscillation/overshoot");
//         telemetry.addData("3", "Add kI to eliminate steady-state error");
//         telemetry.addData("4", "kF helps reach target faster (feed-forward)");
//         telemetry.update();
//     }
    
//     // Helper function to convert RPM to ticks per second
//     private double getTickSpeed(double rpm) {
//         return (rpm * 537.7) / 60.0;
//     }

// }