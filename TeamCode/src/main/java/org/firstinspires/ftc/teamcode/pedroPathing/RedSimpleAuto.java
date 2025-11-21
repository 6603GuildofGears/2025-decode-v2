package org.firstinspires.ftc.teamcode.pedroPathing;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

/**
 * Simple Red Alliance Autonomous - Drive backwards for 2 seconds
 */
@Autonomous(name = "RED - Close Auto", group = "Red")
public class RedSimpleAuto extends OpMode {

    private DcMotor frontLeftDrive;
    private DcMotor frontRightDrive;
    private DcMotor backLeftDrive;
    private DcMotor backRightDrive;
    
    // Shooter/Intake hardware - NOTE: swapped in hardware config
    private DcMotorEx shooter;  // Actually mapped to "intakeMotor"
    private DcMotorEx intake;   // Actually mapped to "shooterMotor"
    private Servo blocker;
    
    // Constants
    private static final double SHOOTER_RPM = 100;
    private static final double TICKS_PER_REV = 28.0;  // GoBilda 5202/5203 encoder
    private static final double RPM_TOLERANCE = 50;
    private static final double BLOCKER_UP_POS = 0.2;    // Open position (inverted)
    private static final double BLOCKER_DOWN_POS = 0.3;  // Closed position (inverted)
    
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

        telemetry.addLine("Simple Drive Test Initialized");
        telemetry.addLine("Will drive backwards for 2 seconds");
        telemetry.update();
    }

    @Override
    public void start() {
        runtime.reset();
        isDriving = true;
    }

    @Override
    public void loop() {
        if (isDriving && runtime.seconds() < 1.75) {
            // Drive backwards and rotate to 45°
            double drive = -0.5;    // Backward
            double turn = 0.25;     // Rotate counterclockwise
            
            frontLeftDrive.setPower(drive);  // -0.75
            frontRightDrive.setPower(drive); // -0.25
            backLeftDrive.setPower(drive);   // -0.75
            backRightDrive.setPower(drive);  // -0.25

            //TURN to shoot, from start
        } else if(isDriving && runtime.seconds() >= 1.75 && runtime.seconds() < 2.375 ) {
            frontLeftDrive.setPower(0.35);
            frontRightDrive.setPower(-0.35);
            backLeftDrive.setPower(0.35);
            backRightDrive.setPower(-0.35);
            //SHOOT SEQUENCE
        } else if(isDriving && runtime.seconds() >= 2.375 && runtime.seconds() < 8.25 ) { // Stop motors after 2 seconds
            frontLeftDrive.setPower(0);
            frontRightDrive.setPower(0);
            backLeftDrive.setPower(0);
            backRightDrive.setPower(0);

            shooter.setVelocity(getTickSpeed(SHOOTER_RPM));
            if (isShooterAtSpeed()) {
                blocker.setPosition(BLOCKER_UP_POS);
                intake.setPower(0.5);
            }
            if(runtime.seconds() >= 7.5 ){
                shooter.setPower(0);
                intake.setPower(0);
                blocker.setPosition(BLOCKER_DOWN_POS);
            }
            //ROTATE to INTAKE
        } else if(isDriving && runtime.seconds() >= 8.25  && runtime.seconds() < 9.25 ) {
            frontLeftDrive.setPower(-0.6);
            backLeftDrive.setPower(-0.6);
            frontRightDrive.setPower(0.6);
            backRightDrive.setPower(0.6);
            //DRIVE to INTAKE1
        } else if(isDriving && runtime.seconds() >= 9.25  && runtime.seconds() < 10.75 ) {
            frontLeftDrive.setPower(-0.625);
            frontRightDrive.setPower(-0.625);
            backLeftDrive.setPower(-0.625);
            backRightDrive.setPower(-0.625);
            if(runtime.seconds() >= 9 ){
                intake.setPower(0.875);
            }
            //BACK to shoot
        } else if(isDriving && runtime.seconds() >= 10.75  && runtime.seconds() < 12.25 ) {
            frontLeftDrive.setPower(0.625);
            frontRightDrive.setPower(0.625);
            backLeftDrive.setPower(0.625);
            backRightDrive.setPower(0.625);
            intake.setPower(0);
            //TURN to shoot, from intake1
        } else if(isDriving && runtime.seconds() >= 12.25  && runtime.seconds() < 13.25 ) {
            frontLeftDrive.setPower(0.6);
            backLeftDrive.setPower(0.6);
            frontRightDrive.setPower(-0.6);
            backRightDrive.setPower(-0.6);
            //Shoot sequence
        } else if(isDriving && runtime.seconds() >= 13.25  && runtime.seconds() < 19.375 ) {
            frontLeftDrive.setPower(0);
            frontRightDrive.setPower(0);
            backLeftDrive.setPower(0);
            backRightDrive.setPower(0);

            shooter.setVelocity(getTickSpeed(SHOOTER_RPM));
            if (isShooterAtSpeed()) {
                blocker.setPosition(BLOCKER_UP_POS);
                intake.setPower(0.5);
            }
            if(runtime.seconds() >= 8.625 ){
                shooter.setPower(0);
                intake.setPower(0);
                blocker.setPosition(BLOCKER_DOWN_POS);
            }
        } else if(isDriving && runtime.seconds() >= 19.375  && runtime.seconds() < 19.75 ) {
            // Turn to bridge Intake 2
            frontLeftDrive.setPower(-0.5);
            backLeftDrive.setPower(-0.5);
            frontRightDrive.setPower(0.5);
            backRightDrive.setPower(0.5);
        } else if(isDriving && runtime.seconds() >= 19.75  && runtime.seconds() < 20.25 ) {
            // Drive to Intake 2 bridge point
            frontLeftDrive.setPower(-0.75);
            frontRightDrive.setPower(-0.75);
            backLeftDrive.setPower(-0.75);
            backRightDrive.setPower(-0.75);
        } else if(isDriving && runtime.seconds() >= 20.25  && runtime.seconds() < 21 ) {
            // Turn to intake 2, from bridge
            frontLeftDrive.setPower(-0.65);
            backLeftDrive.setPower(-0.65);
            frontRightDrive.setPower(0.65);
            backRightDrive.setPower(0.65);
        } else if(isDriving && runtime.seconds() >= 21 && runtime.seconds() < 23.5 ) {
            // Drive to intake 2
            frontLeftDrive.setPower(-0.5);
            frontRightDrive.setPower(-0.5);
            backLeftDrive.setPower(-0.5);
            backRightDrive.setPower(-0.5);
            intake.setPower(0.875);
            //Turn back to shoot, from intake2 compromised
        } else if(isDriving && runtime.seconds() >= 23.5  && runtime.seconds() < 24 ) {
            frontLeftDrive.setPower(0.5);
            backLeftDrive.setPower(0.5);
            frontRightDrive.setPower(-0.5);
            backRightDrive.setPower(-0.5);
            intake.setPower(0);
            //Drive back to shoot
        } else if(isDriving && runtime.seconds() >= 24 && runtime.seconds() < 25.5 ) {
            frontLeftDrive.setPower(0.5);
            frontRightDrive.setPower(0.5);
            backLeftDrive.setPower(0.5);
            backRightDrive.setPower(0.5);
            //Turn to shoot
        } else if(isDriving && runtime.seconds() >= 25.5  && runtime.seconds() < 26.25 ) {
            frontLeftDrive.setPower(0.45);
            backLeftDrive.setPower(0.45);
            frontRightDrive.setPower(-0.45);
            backRightDrive.setPower(-0.45);
            //Shoot sequence
        } else if(isDriving && runtime.seconds() >= 26.25  && runtime.seconds() < 32.625 ) {
            frontLeftDrive.setPower(0);
            frontRightDrive.setPower(0);
            backLeftDrive.setPower(0);
            backRightDrive.setPower(0);

            shooter.setVelocity(getTickSpeed(SHOOTER_RPM));
            if (isShooterAtSpeed()) {
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
}
