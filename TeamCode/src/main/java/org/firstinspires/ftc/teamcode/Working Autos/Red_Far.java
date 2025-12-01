package org.firstinspires.ftc.teamcode.pedroPathing;

// Red Alliance Autonomous
// NOTE: Uses dynamic voltage compensation for consistent performance

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

/**
 * Red Alliance Autonomous - PRIMARY CODE
 * Features dynamic voltage compensation for maximum consistency
 */
@Autonomous(name = "RED - Far Auto", group = "Red")
public class Red_Far extends OpMode {

    private DcMotor frontLeftDrive;
    private DcMotor frontRightDrive;
    private DcMotor backLeftDrive;
    private DcMotor backRightDrive;
    
    // Shooter/Intake hardware - NOTE: swapped in hardware config
    private DcMotorEx shooter;  // Actually mapped to "intakeMotor"
    private DcMotorEx intake;   // Actually mapped to "shooterMotor"
    private Servo blocker;
    
    // Constants
    private static final double SHOOTER_RPM = 3000;
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

        telemetry.addLine("RED PRIMARY Auto Initialized - Dynamic Voltage Compensation Enabled");
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
        if (isDriving && runtime.seconds() < 0.566) {
            // turn to shoot 
            } else if(isDriving && runtime.seconds() >= 0.566 && runtime.seconds() < 5.891 ) {
            frontLeftDrive.setPower(0);
            frontRightDrive.setPower(0);
            backLeftDrive.setPower(0);
            backRightDrive.setPower(0);

            //shoot
            shooter.setVelocity(getTickSpeed(SHOOTER_RPM) * getCurrentVoltageScale());
            if (runtime.seconds() > 1.25) {
                blocker.setPosition(BLOCKER_UP_POS);
                intake.setPower(scalePower(0.3));
            }
            if(runtime.seconds() >= 5.8 ){
                shooter.setPower(0);
                intake.setPower(0);
                blocker.setPosition(BLOCKER_DOWN_POS);

            }
            
            //drive forward
            } else if(isDriving && runtime.seconds() >= 5.891  && runtime.seconds() < 6.33 ) {
            shooter.setPower(0);

            frontLeftDrive.setPower(scalePower(0.448));
            backLeftDrive.setPower(scalePower(0.448));
            frontRightDrive.setPower(scalePower(0.448));
            backRightDrive.setPower(scalePower(0.448));

       
            //rotat to intake 1
        // } else if(isDriving && runtime.seconds() >= 6.016 && runtime.seconds() < 6.767 ) {
        //     frontLeftDrive.setPower(scalePower(-0.4));
        //     frontRightDrive.setPower(scalePower(0.4));
        //     backLeftDrive.setPower(scalePower(-0.4));
        //     backRightDrive.setPower(scalePower(0.4));

        //     //drive to intake
        // } else if(isDriving && runtime.seconds() >= 6.767  && runtime.seconds() < 8.767 ) {
        //     frontLeftDrive.setPower(scalePower(-0.4));
        //     frontRightDrive.setPower(scalePower(-0.4));
        //     backLeftDrive.setPower(scalePower(-0.4));
        //     backRightDrive.setPower(scalePower(-0.4));
        //     intake.setPower(0.75);
        //     //back to shoot
        // } else if(isDriving && runtime.seconds() >= 8.767  && runtime.seconds() < 10.767 ) {
        //     intake.setPower(0);
        //     frontLeftDrive.setPower(scalePower(0.4));
        //     backLeftDrive.setPower(scalePower(0.4));
        //     frontRightDrive.setPower(scalePower(0.4));
        //     backRightDrive.setPower(scalePower(0.4));


        //     //rotae to shoot
        // } else if (isDriving && runtime.seconds()  >= 10.767  && runtime.seconds() <11.517){
        //       frontLeftDrive.setPower(scalePower(0.4));
        //     frontRightDrive.setPower(scalePower(-0.4));
        //     backLeftDrive.setPower(scalePower(0.4));
        //     backRightDrive.setPower(scalePower(-0.4));
        
        
        // // shoot 
        // }else if(isDriving && runtime.seconds() >= 11.517  && runtime.seconds() < 16.767 ) {
        //     frontLeftDrive.setPower(0);
        //     frontRightDrive.setPower(0);
        //     backLeftDrive.setPower(0);
        //     backRightDrive.setPower(0);

        //     shooter.setVelocity(getTickSpeed(SHOOTER_RPM) * getCurrentVoltageScale());
        //     if (runtime.seconds() > 12) {
        //         blocker.setPosition(BLOCKER_UP_POS);
        //         intake.setPower(scalePower(0.4));
        //     }
        //     if(runtime.seconds() >= 16.67 ){
        //         shooter.setPower(0);
        //         intake.setPower(0);
        //         blocker.setPosition(BLOCKER_DOWN_POS);
        //     }
        // } else if(isDriving && runtime.seconds() >= 16.767  && runtime.seconds() < 17.267 ) {
        //     // drive Forward 
        //     shooter.setPower(0);

        //     frontLeftDrive.setPower(scalePower(0.355));
        //     backLeftDrive.setPower(scalePower(0.355));
        //     frontRightDrive.setPower(scalePower(0.355));
        //     backRightDrive.setPower(scalePower(0.355));
        
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

    

