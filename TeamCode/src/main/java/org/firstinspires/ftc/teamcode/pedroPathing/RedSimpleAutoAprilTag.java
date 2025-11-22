package org.firstinspires.ftc.teamcode.pedroPathing;

//APRILTAG EXPERIMENTATION AUTO
// NOTE: This autonomous works well at 13.04 volts battery voltage

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

// Limelight imports
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;

import java.util.List;

/**
 * Red Alliance Autonomous - AprilTag Testing
 */
@Autonomous(name = "RED - AprilTag Test", group = "Red")
public class RedSimpleAutoAprilTag extends OpMode {

    private DcMotor frontLeftDrive;
    private DcMotor frontRightDrive;
    private DcMotor backLeftDrive;
    private DcMotor backRightDrive;
    
    // Shooter/Intake hardware - NOTE: swapped in hardware config
    private DcMotorEx shooter;  // Actually mapped to "intakeMotor"
    private DcMotorEx intake;   // Actually mapped to "shooterMotor"
    private Servo blocker;
    
    // Limelight
    private Limelight3A limelight;
    
    // AprilTag centering constants
    private static final double TURN_SPEED = 0.25;  // Speed for AprilTag alignment
    private static final double STRAFE_SPEED = 0.3; // Speed for left/right adjustment
    private static final double ALIGNMENT_TOLERANCE = 0.05; // Tolerance in meters for X/Y alignment
    
    // Constants
    private static final double SHOOTER_RPM = 2900;
    private static final double TICKS_PER_REV = 28.0;  // GoBilda 5202/5203 encoder
    private static final double RPM_TOLERANCE = 300;
    private static final double BLOCKER_UP_POS = 0.175;    // Open position (inverted)
    private static final double BLOCKER_DOWN_POS = 0.3;  // Closed position (inverted)
    
    // Voltage compensation - tuned for 13.1V battery
    private static final double TARGET_VOLTAGE = 13.1;
    private double voltageScale = 1.0;
    
    private ElapsedTime runtime = new ElapsedTime();
    private boolean isDriving = false;

    @Override
    public void init() {
        // Initialize drive motors
        frontLeftDrive = hardwareMap.get(DcMotor.class, "frontLeftDrive");
        frontRightDrive = hardwareMap.get(DcMotor.class, "frontRightDrive");
        backLeftDrive = hardwareMap.get(DcMotor.class, "backLeftDrive");
        backRightDrive = hardwareMap.get(DcMotor.class, "backRightDrive");

        // Set motor directions
        frontLeftDrive.setDirection(DcMotor.Direction.REVERSE);
        frontRightDrive.setDirection(DcMotor.Direction.REVERSE);
        backLeftDrive.setDirection(DcMotor.Direction.FORWARD);
        backRightDrive.setDirection(DcMotor.Direction.FORWARD);
        
        // Initialize shooter/intake - NOTE: swapped in hardware config
        shooter = hardwareMap.get(DcMotorEx.class, "intakeMotor");  // Actual shooter
        intake = hardwareMap.get(DcMotorEx.class, "shooterMotor");  // Actual intake
        blocker = hardwareMap.get(Servo.class, "blocker");
        
        // Shooter setup
        shooter.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        shooter.setDirection(DcMotorSimple.Direction.REVERSE);
        
        // Intake setup
        intake.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        intake.setDirection(DcMotorSimple.Direction.REVERSE);
        
        // Set blocker to closed position
        blocker.setPosition(BLOCKER_DOWN_POS);
        
        // Initialize Limelight
        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.pipelineSwitch(0);  // Switch to AprilTag pipeline
        limelight.start();
        
        // Calculate voltage compensation scale
        double currentVoltage = hardwareMap.voltageSensor.iterator().next().getVoltage();
        voltageScale = TARGET_VOLTAGE / currentVoltage;

        telemetry.addLine("AprilTag Test Auto Initialized");
        telemetry.addData("Battery Voltage", "%.2f V", currentVoltage);
        telemetry.addData("Voltage Scale", "%.3f", voltageScale);
        telemetry.update();
    }

    @Override
    public void start() {
        runtime.reset();
        isDriving = true;
    }

    @Override
    public void loop() {
        // Check for AprilTag detection and display robot coordinates
        updateAprilTagTelemetry();
        
        if (isDriving && runtime.seconds() < 1.75) {
            // Drive backwards
            frontLeftDrive.setPower(scalePower(-0.3875));
            frontRightDrive.setPower(scalePower(-0.3875));
            backLeftDrive.setPower(scalePower(-0.3875));
            backRightDrive.setPower(scalePower(-0.3875));

            //TURN to shoot, from start
        } else if(isDriving && runtime.seconds() >= 1.75 && runtime.seconds() < 2.375 ) {
            frontLeftDrive.setPower(scalePower(0.285));
            frontRightDrive.setPower(scalePower(-0.285));
            backLeftDrive.setPower(scalePower(0.285));
            backRightDrive.setPower(scalePower(-0.285));
            //SHOOT SEQUENCE
        } else if(isDriving && runtime.seconds() >= 2.375 && runtime.seconds() < 8.25 ) {
            frontLeftDrive.setPower(0);
            frontRightDrive.setPower(0);
            backLeftDrive.setPower(0);
            backRightDrive.setPower(0);
            
            // Center on AprilTag with proportional control
            centerOnAprilTag();

            shooter.setVelocity(getTickSpeed(SHOOTER_RPM));
            if (runtime.seconds() > 4.375) {
                blocker.setPosition(BLOCKER_UP_POS);
                intake.setPower(0.35);
            }
            if(runtime.seconds() >= 7.5 ){
                shooter.setPower(0);
                intake.setPower(0);
                blocker.setPosition(BLOCKER_DOWN_POS);
            }
            //ROTATE to INTAKE
        } else if(isDriving && runtime.seconds() >= 8.25  && runtime.seconds() < 9.25 ) {
            shooter.setPower(0);

            frontLeftDrive.setPower(scalePower(-0.47));
            backLeftDrive.setPower(scalePower(-0.47));
            frontRightDrive.setPower(scalePower(0.47));
            backRightDrive.setPower(scalePower(0.47));

            intake.setPower(1);
            //DRIVE to INTAKE1
        } else if(isDriving && runtime.seconds() >= 9.25  && runtime.seconds() < 10.75 ) {
            frontLeftDrive.setPower(scalePower(-0.4));
            frontRightDrive.setPower(scalePower(-0.4));
            backLeftDrive.setPower(scalePower(-0.4));
            backRightDrive.setPower(scalePower(-0.4));

            //BACK to shoot
        } else if(isDriving && runtime.seconds() >= 10.75  && runtime.seconds() < 12.25 ) {
            frontLeftDrive.setPower(scalePower(0.4));
            frontRightDrive.setPower(scalePower(0.4));
            backLeftDrive.setPower(scalePower(0.4));
            backRightDrive.setPower(scalePower(0.4));
            intake.setPower(0);
            //TURN to shoot, from intake1
        } else if(isDriving && runtime.seconds() >= 12.25  && runtime.seconds() < 13.25 ) {
            frontLeftDrive.setPower(scalePower(0.45));
            backLeftDrive.setPower(scalePower(0.45));
            frontRightDrive.setPower(scalePower(-0.45));
            backRightDrive.setPower(scalePower(-0.45));
            //Shoot sequence
        } else if(isDriving && runtime.seconds() >= 13.25  && runtime.seconds() < 19.375 ) {
            frontLeftDrive.setPower(0);
            frontRightDrive.setPower(0);
            backLeftDrive.setPower(0);
            backRightDrive.setPower(0);
            
            // Center on AprilTag with proportional control
            centerOnAprilTag();

            shooter.setVelocity(getTickSpeed(SHOOTER_RPM));
            if (runtime.seconds() > 15) {
                blocker.setPosition(BLOCKER_UP_POS);
                intake.setPower(0.35);
            }
            if(runtime.seconds() >= 18.5 ){
                shooter.setPower(0);
                intake.setPower(0);
                blocker.setPosition(BLOCKER_DOWN_POS);
            }
        } else if(isDriving && runtime.seconds() >= 19.375  && runtime.seconds() < 19.75 ) {
            // Turn to bridge Intake 2
            shooter.setPower(0);

            frontLeftDrive.setPower(scalePower(-0.35));
            backLeftDrive.setPower(scalePower(-0.35));
            frontRightDrive.setPower(scalePower(0.35));
            backRightDrive.setPower(scalePower(0.35));
        } else if(isDriving && runtime.seconds() >= 19.75  && runtime.seconds() < 20.25 ) {
            // Drive to Intake 2 bridge point
            frontLeftDrive.setPower(scalePower(-0.5));
            frontRightDrive.setPower(scalePower(-0.5));
            backLeftDrive.setPower(scalePower(-0.5));
            backRightDrive.setPower(scalePower(-0.5));
        } else if(isDriving && runtime.seconds() >= 20.25  && runtime.seconds() < 21 ) {
            // Turn to intake 2, from bridge
            frontLeftDrive.setPower(scalePower(-0.5));
            backLeftDrive.setPower(scalePower(-0.5));
            frontRightDrive.setPower(scalePower(0.5));
            backRightDrive.setPower(scalePower(0.5));
        } else if(isDriving && runtime.seconds() >= 21 && runtime.seconds() < 23.5 ) {
            // Drive to intake 2
            frontLeftDrive.setPower(scalePower(-0.5));
            frontRightDrive.setPower(scalePower(-0.5));
            backLeftDrive.setPower(scalePower(-0.5));
            backRightDrive.setPower(scalePower(-0.5));
            intake.setPower(0.875);
            //Turn back to shoot, from intake2 compromised
        } else if(isDriving && runtime.seconds() >= 23.5  && runtime.seconds() < 24 ) {
            frontLeftDrive.setPower(scalePower(0.5));
            backLeftDrive.setPower(scalePower(0.5));
            frontRightDrive.setPower(scalePower(-0.5));
            backRightDrive.setPower(scalePower(-0.5));
            intake.setPower(0);
            //Drive back to shoot
        } else if(isDriving && runtime.seconds() >= 24 && runtime.seconds() < 25.5 ) {
            frontLeftDrive.setPower(scalePower(0.5));
            frontRightDrive.setPower(scalePower(0.5));
            backLeftDrive.setPower(scalePower(0.5));
            backRightDrive.setPower(scalePower(0.5));
            //Turn to shoot
        } else if(isDriving && runtime.seconds() >= 25.5  && runtime.seconds() < 26.25 ) {
            frontLeftDrive.setPower(scalePower(0.45));
            backLeftDrive.setPower(scalePower(0.45));
            frontRightDrive.setPower(scalePower(-0.45));
            backRightDrive.setPower(scalePower(-0.45));
            //Shoot sequence
        } else if(isDriving && runtime.seconds() >= 26.25  && runtime.seconds() < 32.625 ) {
            frontLeftDrive.setPower(0);
            frontRightDrive.setPower(0);
            backLeftDrive.setPower(0);
            backRightDrive.setPower(0);
            
            // Center on AprilTag with proportional control
            centerOnAprilTag();

            shooter.setVelocity(getTickSpeed(SHOOTER_RPM));
            if (runtime.seconds() > 28) {
                blocker.setPosition(BLOCKER_UP_POS);
                intake.setPower(0.5);
            }
            if(runtime.seconds() >= 31.875 ){
                shooter.setPower(0);
                intake.setPower(0);
                blocker.setPosition(BLOCKER_DOWN_POS);
            }
        }
        
        else if (isDriving) {
            // Stop motors after 2 seconds
            frontLeftDrive.setPower(0);
            frontRightDrive.setPower(0);
            backLeftDrive.setPower(0);
            backRightDrive.setPower(0);
            isDriving = false;
        }

        // Telemetry
        telemetry.addData("Status", isDriving ? "Driving" : "Stopped");
        telemetry.addData("Time", "%.2f s", runtime.seconds());
        telemetry.update();
    }

    /**
     * Centers the robot on an AprilTag using Limelight feedback.
     * @return true if centered or no tag detected (stop motors), false if actively centering
     */
    private boolean centerOnAprilTag() {
        // DISABLED - AprilTag centering turned off
        return true;
        
        /* ORIGINAL CODE - DISABLED
        LLResult result = limelight.getLatestResult();
        if (result == null || !result.isValid()) {
            return true; // No tag detected, use manual control
        }

        List<LLResultTypes.FiducialResult> fiducials = result.getFiducialResults();
        if (fiducials.isEmpty()) {
            return true; // No tag detected, use manual control
        }

        // Get robot position from first detected tag
        LLResultTypes.FiducialResult tag = fiducials.get(0);
        Pose3D robotPose = tag.getRobotPoseFieldSpace();

        // Calculate offsets (target is tag position at 0,0)
        double xOffset = robotPose.getPosition().x;
        double yOffset = robotPose.getPosition().y;
        double headingError = Math.toDegrees(robotPose.getOrientation().getYaw());

        // Deadband to prevent tiny oscillations
        if (Math.abs(yOffset) < 0.02) yOffset = 0;
        if (Math.abs(headingError) < 2.0) headingError = 0;

        // Check if centered within tolerance
        if (Math.abs(xOffset) < ALIGNMENT_TOLERANCE && 
            Math.abs(yOffset) < ALIGNMENT_TOLERANCE &&
            Math.abs(headingError) < 5.0) {
            return true; // Centered, allow manual control to stop motors
        }

        // Apply proportional corrections (smaller multipliers for smoother control)
        double turn = headingError * 0.015;  // Proportional to heading error
        double strafe = yOffset * 0.5;        // Proportional to lateral error
        
        // Limit maximum correction speeds
        turn = Math.max(-TURN_SPEED, Math.min(TURN_SPEED, turn));
        strafe = Math.max(-STRAFE_SPEED, Math.min(STRAFE_SPEED, strafe));

        // Apply mecanum drive with centering corrections
        frontLeftDrive.setPower(scalePower(-strafe - turn));
        frontRightDrive.setPower(scalePower(strafe + turn));
        backLeftDrive.setPower(scalePower(strafe - turn));
        backRightDrive.setPower(scalePower(-strafe + turn));

        return false; // Still centering, motors controlled by this method
        */
    }

    @Override
    public void stop() {
        frontLeftDrive.setPower(0);
        frontRightDrive.setPower(0);
        backLeftDrive.setPower(0);
        backRightDrive.setPower(0);
        shooter.setVelocity(0);
        intake.setPower(0);
    }
    
    /**
     * Utility: Convert RPM to ticks per second
     */
    private double getTickSpeed(double rpm) {
        return rpm * TICKS_PER_REV / 60;
    }
    
    /**
     * Check if shooter is at target speed
     */
    private boolean isShooterAtSpeed() {
        double currentRPM = shooter.getVelocity() * 60 / TICKS_PER_REV;
        return Math.abs(currentRPM - SHOOTER_RPM) < RPM_TOLERANCE;
    }
    
    /**
     * Scale motor power for voltage compensation
     */
    private double scalePower(double power) {
        return power * voltageScale;
    }
    
    /**
     * Update telemetry with AprilTag detection and robot coordinates
     */
    private void updateAprilTagTelemetry() {
        LLResult result = limelight.getLatestResult();
        
        if (result != null && result.isValid()) {
            List<LLResultTypes.FiducialResult> fiducials = result.getFiducialResults();
            
            if (!fiducials.isEmpty()) {
                // Get the first detected AprilTag
                LLResultTypes.FiducialResult tag = fiducials.get(0);
                
                // Get robot pose in field space
                Pose3D robotPose = tag.getRobotPoseFieldSpace();
                
                // Display robot coordinates on Driver Hub
                telemetry.addLine("=== APRILTAG DETECTED ===");
                telemetry.addData("Tag ID", tag.getFiducialId());
                telemetry.addData("Robot X (meters)", "%.3f", robotPose.getPosition().x);
                telemetry.addData("Robot Y (meters)", "%.3f", robotPose.getPosition().y);
                telemetry.addData("Robot Z (meters)", "%.3f", robotPose.getPosition().z);
                telemetry.addData("Robot Heading (deg)", "%.1f", Math.toDegrees(robotPose.getOrientation().getYaw()));
            } else {
                telemetry.addLine("No AprilTag detected");
            }
        } else {
            telemetry.addLine("No Limelight data");
        }
    }
}
