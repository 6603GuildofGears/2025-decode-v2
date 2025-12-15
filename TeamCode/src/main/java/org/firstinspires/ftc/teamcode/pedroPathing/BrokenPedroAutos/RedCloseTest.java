package org.firstinspires.ftc.teamcode.pedroPathing;

// Red Alliance Autonomous
// NOTE: Uses dynamic voltage compensation for consistent performance

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

/**
 * Red Alliance Autonomous - Close Test (2X Speed)
 * Features dynamic voltage compensation for maximum consistency
 */

@Disabled
@Autonomous(name = "RED - Close Test", group = "Red")
public class RedCloseTest extends OpMode {

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
    private static final double BLOCKER_DOWN_POS = 0.32;  // Closed position (inverted)
    
    // Voltage compensation - tuned for 13.8V battery
    private static final double TARGET_VOLTAGE = 13.8;
    private static final double MIN_VOLTAGE = 11.5;  // Tighter minimum for consistency
    private static final double MAX_VOLTAGE = 14.5;  // Maximum battery voltage when fully charged
    
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

        telemetry.addLine("RED Close Test Auto Initialized - Dynamic Voltage Compensation Enabled");
        telemetry.addData("Battery Voltage", "%.2f V", hardwareMap.voltageSensor.iterator().next().getVoltage());
        telemetry.addData("Target Voltage", "%.2f V", TARGET_VOLTAGE);
        telemetry.update();
    }

    @Override
    public void start() {
        runtime.reset();
        isDriving = true;
    }

    @Override
    public void loop() {
        if (isDriving && runtime.seconds() < 0.793) {
            // Drive backwards (2X power)
            frontLeftDrive.setPower(scalePower(-0.775));
            frontRightDrive.setPower(scalePower(-0.775));
            backLeftDrive.setPower(scalePower(-0.775));
            backRightDrive.setPower(scalePower(-0.775));

            //TURN to shoot, from start (2X power)
        } else if(isDriving && runtime.seconds() >= 0.793 && runtime.seconds() < 1.076 ) {
            frontLeftDrive.setPower(scalePower(0.57));
            frontRightDrive.setPower(scalePower(-0.57));
            backLeftDrive.setPower(scalePower(0.57));
            backRightDrive.setPower(scalePower(-0.57));
            //SHOOT SEQUENCE
        } else if(isDriving && runtime.seconds() >= 1.076 && runtime.seconds() < 3.7385 ) {
            frontLeftDrive.setPower(0);
            frontRightDrive.setPower(0);
            backLeftDrive.setPower(0);
            backRightDrive.setPower(0);

            shooter.setVelocity(getTickSpeed(SHOOTER_RPM) * getCurrentVoltageScale());
            if (runtime.seconds() > 2.1) {
                blocker.setPosition(BLOCKER_UP_POS);
                intake.setPower(scalePower(0.7));
            }
            if(runtime.seconds() >= 3.397 ){
                shooter.setPower(0);
                intake.setPower(0);
                blocker.setPosition(BLOCKER_DOWN_POS);
            }
            //ROTATE to INTAKE (2X power)
        } else if(isDriving && runtime.seconds() >= 3.7385  && runtime.seconds() < 4.1915 ) {
            shooter.setPower(0);

            frontLeftDrive.setPower(scalePower(-0.896));
            backLeftDrive.setPower(scalePower(-0.896));
            frontRightDrive.setPower(scalePower(0.896));
            backRightDrive.setPower(scalePower(0.896));

            intake.setPower(scalePower(1));
            //DRIVE to INTAKE1 (2X power)
        } else if(isDriving && runtime.seconds() >= 4.1915  && runtime.seconds() < 4.871 ) {
            frontLeftDrive.setPower(scalePower(-0.8));
            frontRightDrive.setPower(scalePower(-0.8));
            backLeftDrive.setPower(scalePower(-0.8));
            backRightDrive.setPower(scalePower(-0.8));

            //BACK to shoot (2X power)
        } else if(isDriving && runtime.seconds() >= 4.871  && runtime.seconds() < 5.5505 ) {
            frontLeftDrive.setPower(scalePower(0.8));
            frontRightDrive.setPower(scalePower(0.8));
            backLeftDrive.setPower(scalePower(0.8));
            backRightDrive.setPower(scalePower(0.8));
            intake.setPower(0);
            //TURN to shoot, from intake1 (2X power)
        } else if(isDriving && runtime.seconds() >= 5.5505  && runtime.seconds() < 6.0035 ) {
            frontLeftDrive.setPower(scalePower(0.875));
            backLeftDrive.setPower(scalePower(0.92));
            frontRightDrive.setPower(scalePower(-0.92));
            backRightDrive.setPower(scalePower(-0.875));
            //Shoot sequence
        } else if(isDriving && runtime.seconds() >= 6.0035  && runtime.seconds() < 8.7755 ) {
            frontLeftDrive.setPower(0);
            frontRightDrive.setPower(0);
            backLeftDrive.setPower(0);
            backRightDrive.setPower(0);

            shooter.setVelocity(getTickSpeed(SHOOTER_RPM) * getCurrentVoltageScale());
            if (runtime.seconds() > 6.85) {
                blocker.setPosition(BLOCKER_UP_POS);
                intake.setPower(scalePower(0.7));
            }
            if(runtime.seconds() >= 8.379 ){
                shooter.setPower(0);
                intake.setPower(0);
                blocker.setPosition(BLOCKER_DOWN_POS);
            }
        } else if(isDriving && runtime.seconds() >= 8.7755  && runtime.seconds() < 8.9455 ) {
            // Turn to bridge Intake 2 (2X power)
            shooter.setPower(0);

            frontLeftDrive.setPower(scalePower(-0.71));
            backLeftDrive.setPower(scalePower(-0.71));
            frontRightDrive.setPower(scalePower(0.71));
            backRightDrive.setPower(scalePower(0.71));
        } else if(isDriving && runtime.seconds() >= 8.9455  && runtime.seconds() < 9.172 ) {
            // Drive to Intake 2 bridge point (2X power)
            frontLeftDrive.setPower(scalePower(-1.0));
            frontRightDrive.setPower(scalePower(-1.0));
            backLeftDrive.setPower(scalePower(-1.0));
            backRightDrive.setPower(scalePower(-1.0));
        } else if(isDriving && runtime.seconds() >= 9.172  && runtime.seconds() < 9.5115 ) {
            // Turn to intake 2, from bridge (2X power)
            frontLeftDrive.setPower(scalePower(-0.91));
            backLeftDrive.setPower(scalePower(-0.91));
            frontRightDrive.setPower(scalePower(0.91));
            backRightDrive.setPower(scalePower(0.91));
        } else if(isDriving && runtime.seconds() >= 9.5115 && runtime.seconds() < 10.644 ) {
            // Drive to intake 2 (2X power)
            frontLeftDrive.setPower(scalePower(-0.6));
            frontRightDrive.setPower(scalePower(-0.6));
            backLeftDrive.setPower(scalePower(-0.6));
            backRightDrive.setPower(scalePower(-0.6));
            intake.setPower(scalePower(1.0));
            //Turn back to shoot, from intake2 compromised (2X power)
        } else if(isDriving && runtime.seconds() >= 10.644  && runtime.seconds() < 10.8705 ) {
            frontLeftDrive.setPower(scalePower(0.4));
            backLeftDrive.setPower(scalePower(0.4));
            frontRightDrive.setPower(scalePower(-0.4));
            backRightDrive.setPower(scalePower(-0.4));
            intake.setPower(0);
            //Drive back to shoot (2X power)
        } else if(isDriving && runtime.seconds() >= 10.8705 && runtime.seconds() < 11.5495 ) {
            frontLeftDrive.setPower(scalePower(0.84));
            frontRightDrive.setPower(scalePower(0.84));
            backLeftDrive.setPower(scalePower(0.84));
            backRightDrive.setPower(scalePower(0.84));
            //Turn to shoot (2X power)
        } else if(isDriving && runtime.seconds() >= 11.5495  && runtime.seconds() < 11.889 ) {
            frontLeftDrive.setPower(scalePower(0.932));
            backLeftDrive.setPower(scalePower(0.932));
            frontRightDrive.setPower(scalePower(-0.932));
            backRightDrive.setPower(scalePower(-0.932));
            //Shoot sequence
        } else if(isDriving && runtime.seconds() >= 11.889  && runtime.seconds() < 14.7725 ) {
            frontLeftDrive.setPower(0);
            frontRightDrive.setPower(0);
            backLeftDrive.setPower(0);
            backRightDrive.setPower(0);

            shooter.setVelocity(getTickSpeed(SHOOTER_RPM) * getCurrentVoltageScale());
            if (runtime.seconds() > 12.8) {
                blocker.setPosition(BLOCKER_UP_POS);
                intake.setPower(scalePower(0.7));
            }
            if(runtime.seconds() >= 14.436 ){
                shooter.setPower(0);
                intake.setPower(0);
                blocker.setPosition(BLOCKER_DOWN_POS);
            }
        } else if (isDriving) {
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
        telemetry.addData("Battery Voltage", "%.2f V", hardwareMap.voltageSensor.iterator().next().getVoltage());
        telemetry.addData("Voltage Scale", "%.3f", getCurrentVoltageScale());
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
     * Scale motor power for voltage compensation (dynamic)
     */
    private double scalePower(double power) {
        return power * getCurrentVoltageScale();
    }
    
    /**
     * Get current voltage compensation scale factor with safety clamping
     * Full precision compensation for maximum positioning consistency
     */
    private double getCurrentVoltageScale() {
        double currentVoltage = hardwareMap.voltageSensor.iterator().next().getVoltage();
        
        // Clamp voltage to safe range to prevent extreme compensation
        currentVoltage = Math.max(MIN_VOLTAGE, Math.min(MAX_VOLTAGE, currentVoltage));
        
        // Calculate precise compensation ratio (no artificial scale limiting)
        // At 14.5V: 12.5/14.5 = 0.862069... (full precision)
        // At 11.5V: 12.5/11.5 = 1.086956... (full precision)
        return TARGET_VOLTAGE / currentVoltage;
    }
}
