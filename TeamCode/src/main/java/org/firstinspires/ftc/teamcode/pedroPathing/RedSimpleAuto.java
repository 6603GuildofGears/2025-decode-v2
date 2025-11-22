package org.firstinspires.ftc.teamcode.pedroPathing;

//TOY WITH THIS AUTO
// NOTE: This autonomous works well at 13.04 volts battery voltage

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
        
        // Calculate voltage compensation scale
        double currentVoltage = hardwareMap.voltageSensor.iterator().next().getVoltage();
        voltageScale = TARGET_VOLTAGE / currentVoltage;

        telemetry.addLine("Simple Drive Test Initialized");
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
        if (isDriving && runtime.seconds() < 1.75) {
            // Drive backwards and rotate to 45°
            double drive = -0.3875;    // Backward
            double turn = 0.25;     // Rotate counterclockwise
            
            frontLeftDrive.setPower(scalePower(drive));
            frontRightDrive.setPower(scalePower(drive));
            backLeftDrive.setPower(scalePower(drive));
            backRightDrive.setPower(scalePower(drive));

            //TURN to shoot, from start
        } else if(isDriving && runtime.seconds() >= 1.75 && runtime.seconds() < 2.375 ) {
            frontLeftDrive.setPower(scalePower(0.285));
            frontRightDrive.setPower(scalePower(-0.285));
            backLeftDrive.setPower(scalePower(0.285));
            backRightDrive.setPower(scalePower(-0.285));
            //SHOOT SEQUENCE
        } else if(isDriving && runtime.seconds() >= 2.375 && runtime.seconds() < 8.25 ) { // Stop motors after 2 seconds
            frontLeftDrive.setPower(0);
            frontRightDrive.setPower(0);
            backLeftDrive.setPower(0);
            backRightDrive.setPower(0);

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

            frontLeftDrive.setPower(scalePower(-0.495));
            backLeftDrive.setPower(scalePower(-0.495));
            frontRightDrive.setPower(scalePower(0.495));
            backRightDrive.setPower(scalePower(0.495));

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
            frontLeftDrive.setPower(scalePower(0.4375));
            backLeftDrive.setPower(scalePower(0.4375));
            frontRightDrive.setPower(scalePower(-0.4375));
            backRightDrive.setPower(scalePower(-0.4375));
            //Shoot sequence
        } else if(isDriving && runtime.seconds() >= 13.25  && runtime.seconds() < 19.375 ) {
            frontLeftDrive.setPower(0);
            frontRightDrive.setPower(0);
            backLeftDrive.setPower(0);
            backRightDrive.setPower(0);

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

            frontLeftDrive.setPower(scalePower(-0.355));
            backLeftDrive.setPower(scalePower(-0.355));
            frontRightDrive.setPower(scalePower(0.355));
            backRightDrive.setPower(scalePower(0.355));
        } else if(isDriving && runtime.seconds() >= 19.75  && runtime.seconds() < 20.25 ) {
            // Drive to Intake 2 bridge point
            frontLeftDrive.setPower(scalePower(-0.5));
            frontRightDrive.setPower(scalePower(-0.5));
            backLeftDrive.setPower(scalePower(-0.5));
            backRightDrive.setPower(scalePower(-0.5));
        } else if(isDriving && runtime.seconds() >= 20.25  && runtime.seconds() < 21 ) {
            // Turn to intake 2, from bridge
            frontLeftDrive.setPower(scalePower(-0.455));
            backLeftDrive.setPower(scalePower(-0.455));
            frontRightDrive.setPower(scalePower(0.455));
            backRightDrive.setPower(scalePower(0.455));
        } else if(isDriving && runtime.seconds() >= 21 && runtime.seconds() < 23.5 ) {
            // Drive to intake 2
            frontLeftDrive.setPower(scalePower(-0.3));
            frontRightDrive.setPower(scalePower(-0.3));
            backLeftDrive.setPower(scalePower(-0.3));
            backRightDrive.setPower(scalePower(-0.3));
            intake.setPower(0.875);
            //Turn back to shoot, from intake2 compromised
        } else if(isDriving && runtime.seconds() >= 23.5  && runtime.seconds() < 24 ) {
            frontLeftDrive.setPower(scalePower(0.2));
            backLeftDrive.setPower(scalePower(0.2));
            frontRightDrive.setPower(scalePower(-0.2));
            backRightDrive.setPower(scalePower(-0.2));
            intake.setPower(0);
            //Drive back to shoot
        } else if(isDriving && runtime.seconds() >= 24 && runtime.seconds() < 25.5 ) {
            frontLeftDrive.setPower(scalePower(0.42));
            frontRightDrive.setPower(scalePower(0.42));
            backLeftDrive.setPower(scalePower(0.42));
            backRightDrive.setPower(scalePower(0.42));
            //Turn to shoot
        } else if(isDriving && runtime.seconds() >= 25.5  && runtime.seconds() < 26.25 ) {
            frontLeftDrive.setPower(scalePower(0.4665));
            backLeftDrive.setPower(scalePower(0.4665));
            frontRightDrive.setPower(scalePower(-0.4665));
            backRightDrive.setPower(scalePower(-0.4665));
            //Shoot sequence
        } else if(isDriving && runtime.seconds() >= 26.25  && runtime.seconds() < 32.625 ) {
            frontLeftDrive.setPower(0);
            frontRightDrive.setPower(0);
            backLeftDrive.setPower(0);
            backRightDrive.setPower(0);

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
}
