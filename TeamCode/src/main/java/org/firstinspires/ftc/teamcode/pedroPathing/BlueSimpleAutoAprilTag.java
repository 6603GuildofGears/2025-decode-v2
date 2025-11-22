package org.firstinspires.ftc.teamcode.pedroPathing;

// Blue Alliance Autonomous
// NOTE: Uses dynamic voltage compensation for consistent performance

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

/**
 * Blue Alliance Autonomous - PRIMARY CODE
 * Features dynamic voltage compensation for maximum consistency
 */
@Autonomous(name = "BLUE - PRIMARY Auto", group = "Blue")
public class BlueSimpleAutoAprilTag extends OpMode {

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

        telemetry.addLine("BLUE PRIMARY Auto Initialized - Dynamic Voltage Compensation Enabled");
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
        if (isDriving && runtime.seconds() < 1.586) {
            // Drive backwards
            frontLeftDrive.setPower(scalePower(-0.3875));
            frontRightDrive.setPower(scalePower(-0.3875));
            backLeftDrive.setPower(scalePower(-0.3875));
            backRightDrive.setPower(scalePower(-0.3875));

            //TURN to shoot, from start (INVERTED)
        } else if(isDriving && runtime.seconds() >= 1.586 && runtime.seconds() < 2.152 ) {
            frontLeftDrive.setPower(scalePower(-0.285));
            frontRightDrive.setPower(scalePower(0.285));
            backLeftDrive.setPower(scalePower(-0.285));
            backRightDrive.setPower(scalePower(0.285));
            //SHOOT SEQUENCE
        } else if(isDriving && runtime.seconds() >= 2.152 && runtime.seconds() < 7.477 ) {
            frontLeftDrive.setPower(0);
            frontRightDrive.setPower(0);
            backLeftDrive.setPower(0);
            backRightDrive.setPower(0);

            shooter.setVelocity(getTickSpeed(SHOOTER_RPM) * getCurrentVoltageScale());
            if (runtime.seconds() > 4.2) {
                blocker.setPosition(BLOCKER_UP_POS);
                intake.setPower(scalePower(0.35));
            }
            if(runtime.seconds() >= 6.794 ){
                shooter.setPower(0);
                intake.setPower(0);
                blocker.setPosition(BLOCKER_DOWN_POS);
            }
            //ROTATE to INTAKE (INVERTED)
        } else if(isDriving && runtime.seconds() >= 7.477  && runtime.seconds() < 8.383 ) {
            shooter.setPower(0);

            frontLeftDrive.setPower(scalePower(0.448));
            backLeftDrive.setPower(scalePower(0.448));
            frontRightDrive.setPower(scalePower(-0.448));
            backRightDrive.setPower(scalePower(-0.448));

            intake.setPower(scalePower(1));
            //DRIVE to INTAKE1
        } else if(isDriving && runtime.seconds() >= 8.383  && runtime.seconds() < 9.742 ) {
            frontLeftDrive.setPower(scalePower(-0.4));
            frontRightDrive.setPower(scalePower(-0.4));
            backLeftDrive.setPower(scalePower(-0.4));
            backRightDrive.setPower(scalePower(-0.4));

            //BACK to shoot
        } else if(isDriving && runtime.seconds() >= 9.742  && runtime.seconds() < 11.101 ) {
            frontLeftDrive.setPower(scalePower(0.4));
            frontRightDrive.setPower(scalePower(0.4));
            backLeftDrive.setPower(scalePower(0.4));
            backRightDrive.setPower(scalePower(0.4));
            intake.setPower(0);
            //TURN to shoot, from intake1 (INVERTED)
        } else if(isDriving && runtime.seconds() >= 11.101  && runtime.seconds() < 12.007 ) {
            frontLeftDrive.setPower(scalePower(-0.4375));
            backLeftDrive.setPower(scalePower(-0.46));
            frontRightDrive.setPower(scalePower(0.46));
            backRightDrive.setPower(scalePower(0.4375));
            //Shoot sequence
        } else if(isDriving && runtime.seconds() >= 12.007  && runtime.seconds() < 17.551 ) {
            frontLeftDrive.setPower(0);
            frontRightDrive.setPower(0);
            backLeftDrive.setPower(0);
            backRightDrive.setPower(0);

            shooter.setVelocity(getTickSpeed(SHOOTER_RPM) * getCurrentVoltageScale());
            if (runtime.seconds() > 13.7) {
                blocker.setPosition(BLOCKER_UP_POS);
                intake.setPower(scalePower(0.35));
            }
            if(runtime.seconds() >= 16.758 ){
                shooter.setPower(0);
                intake.setPower(0);
                blocker.setPosition(BLOCKER_DOWN_POS);
            }
        } else if(isDriving && runtime.seconds() >= 17.551  && runtime.seconds() < 17.891 ) {
            // Turn to bridge Intake 2 (INVERTED)
            shooter.setPower(0);

            frontLeftDrive.setPower(scalePower(0.355));
            backLeftDrive.setPower(scalePower(0.355));
            frontRightDrive.setPower(scalePower(-0.355));
            backRightDrive.setPower(scalePower(-0.355));
        } else if(isDriving && runtime.seconds() >= 17.891  && runtime.seconds() < 18.344 ) {
            // Drive to Intake 2 bridge point
            frontLeftDrive.setPower(scalePower(-0.5));
            frontRightDrive.setPower(scalePower(-0.5));
            backLeftDrive.setPower(scalePower(-0.5));
            backRightDrive.setPower(scalePower(-0.5));
        } else if(isDriving && runtime.seconds() >= 18.344  && runtime.seconds() < 19.023 ) {
            // Turn to intake 2, from bridge (INVERTED)
            frontLeftDrive.setPower(scalePower(0.455));
            backLeftDrive.setPower(scalePower(0.455));
            frontRightDrive.setPower(scalePower(-0.455));
            backRightDrive.setPower(scalePower(-0.455));
        } else if(isDriving && runtime.seconds() >= 19.023 && runtime.seconds() < 21.288 ) {
            // Drive to intake 2
            frontLeftDrive.setPower(scalePower(-0.3));
            frontRightDrive.setPower(scalePower(-0.3));
            backLeftDrive.setPower(scalePower(-0.3));
            backRightDrive.setPower(scalePower(-0.3));
            intake.setPower(scalePower(0.875));
            //Turn back to shoot, from intake2 compromised (INVERTED)
        } else if(isDriving && runtime.seconds() >= 21.288  && runtime.seconds() < 21.741 ) {
            frontLeftDrive.setPower(scalePower(-0.2));
            backLeftDrive.setPower(scalePower(-0.2));
            frontRightDrive.setPower(scalePower(0.2));
            backRightDrive.setPower(scalePower(0.2));
            intake.setPower(0);
            //Drive back to shoot
        } else if(isDriving && runtime.seconds() >= 21.741 && runtime.seconds() < 23.099 ) {
            frontLeftDrive.setPower(scalePower(0.42));
            frontRightDrive.setPower(scalePower(0.42));
            backLeftDrive.setPower(scalePower(0.42));
            backRightDrive.setPower(scalePower(0.42));
            //Turn to shoot (INVERTED)
        } else if(isDriving && runtime.seconds() >= 23.099  && runtime.seconds() < 23.778 ) {
            frontLeftDrive.setPower(scalePower(-0.466));
            backLeftDrive.setPower(scalePower(-0.466));
            frontRightDrive.setPower(scalePower(0.466));
            backRightDrive.setPower(scalePower(0.466));
            //Shoot sequence
        } else if(isDriving && runtime.seconds() >= 23.778  && runtime.seconds() < 29.545 ) {
            frontLeftDrive.setPower(0);
            frontRightDrive.setPower(0);
            backLeftDrive.setPower(0);
            backRightDrive.setPower(0);

            shooter.setVelocity(getTickSpeed(SHOOTER_RPM) * getCurrentVoltageScale());
            if (runtime.seconds() > 25.6) {
                blocker.setPosition(BLOCKER_UP_POS);
                intake.setPower(scalePower(0.35));
            }
            if(runtime.seconds() >= 28.872 ){
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
