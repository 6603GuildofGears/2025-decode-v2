package org.firstinspires.ftc.teamcode;

// Imports for Dashboard
import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

// Import the Pedro Pathing classes
import com.pedropathing.follower.Follower;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
// Custom Limelight class
import org.firstinspires.ftc.teamcode.Limelight;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;


@TeleOp(name = "Rnago", group = "Robot")
public class Rnago extends OpMode {

    // Hardware variables - Use DcMotorEx for all motors for consistency
    private DcMotorEx frontLeftDrive, frontRightDrive, backLeftDrive, backRightDrive;
    private DcMotorEx intake;
    private DcMotorEx shooter;
    private Servo blocker;
    private IMU imu;

    // Pedro Pathing Follower
    private Follower follower;

    // Limelight Variables
    private Limelight limelight;
    private boolean isNormalView = false, areLedsOn = false;
    private final ElapsedTime cameraToggleTimer = new ElapsedTime();
    private final ElapsedTime ledToggleTimer = new ElapsedTime();
    private static final int EXPOSURE_NORMAL = 70;
    private static final int EXPOSURE_TRACKING = 5;

    // Constants
    private final double TICKS_PER_REV = 28;

    @Override
    public void init() {
        // Set up FTC Dashboard
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        // Hardware mapping
        frontLeftDrive = hardwareMap.get(DcMotorEx.class, "frontLeftDrive");
        frontRightDrive = hardwareMap.get(DcMotorEx.class, "frontRightDrive");
        backLeftDrive = hardwareMap.get(DcMotorEx.class, "backLeftDrive");
        backRightDrive = hardwareMap.get(DcMotorEx.class, "backRightDrive");
        intake = hardwareMap.get(DcMotorEx.class, "intakeMotor");
        shooter = hardwareMap.get(DcMotorEx.class, "shooterMotor");
        blocker = hardwareMap.get(Servo.class, "blocker");
        imu = hardwareMap.get(IMU.class, "imu");

        // Initialize Pedro Pathing follower

        // Drivetrain setup
        frontLeftDrive.setDirection(DcMotorSimple.Direction.REVERSE);
        backLeftDrive.setDirection(DcMotorSimple.Direction.REVERSE);

        frontLeftDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        frontRightDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backLeftDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backRightDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        // Intake and Shooter setup
        intake.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        intake.setDirection(DcMotorSimple.Direction.REVERSE);

        shooter.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        // IMU setup
        RevHubOrientationOnRobot.LogoFacingDirection logoDirection = RevHubOrientationOnRobot.LogoFacingDirection.FORWARD;
        RevHubOrientationOnRobot.UsbFacingDirection usbDirection = RevHubOrientationOnRobot.UsbFacingDirection.DOWN;
        RevHubOrientationOnRobot orientationOnRobot = new RevHubOrientationOnRobot(logoDirection, usbDirection);
        imu.initialize(new IMU.Parameters(orientationOnRobot));

        // Limelight setup
        limelight = new Limelight();
        limelight.setCameraMode(0); // Set to Vision Processor mode
        limelight.setExposure(EXPOSURE_TRACKING);
        limelight.setLedMode(1); // Force LEDs off
        limelight.setPipeline(0);

        telemetry.addLine("Initialization Complete");
        telemetry.update();
    }

    @Override
    public void start() {
        blocker.setPosition(0.3);
        // Reset timers to prevent immediate toggling
        cameraToggleTimer.reset();
        ledToggleTimer.reset();
    }

    @Override
    public void loop() {
        // Update odometry localization
        follower.update();

        // Handle all robot controls
        handleLimelightToggles();
        handleSubsystemControls();
        handleDriveControls();

        // Display telemetry
        displayTelemetry();
    }

    /**
     * Handles the logic for toggling Limelight camera view and LEDs using gamepad 1.
     */
    private void handleLimelightToggles() {
        // Toggle camera view between normal and tracking
        if (gamepad1.y && cameraToggleTimer.seconds() > 0.5) {
            isNormalView = !isNormalView;
            if (isNormalView) {
                limelight.setExposure(EXPOSURE_NORMAL);
                telemetry.speak("Normal View");
            } else {
                limelight.setExposure(EXPOSURE_TRACKING);
                telemetry.speak("Tracking View");
            }
            cameraToggleTimer.reset();
        }

        // Toggle Limelight LEDs on and off
        if (gamepad1.a && ledToggleTimer.seconds() > 0.5) {
            areLedsOn = !areLedsOn;
            if (areLedsOn) {
                limelight.setLedMode(3); // Force On
                telemetry.speak("L E D On");
            } else {
                limelight.setLedMode(1); // Force Off
                telemetry.speak("L E D Off");
            }
            ledToggleTimer.reset();
        }
    }

    /**
     * Handles the logic for the intake, shooter, and blocker mechanisms using gamepad 2.
     */
    private void handleSubsystemControls() {
        // Intake motor control (simplified and corrected)
        double rpm = 3500;
        if (gamepad2.right_bumper) {
            intake.setVelocity(getTickSpeed(rpm));
        } else if (gamepad2.right_trigger > 0.1) { // Added a deadzone for the trigger
            intake.setVelocity(getTickSpeed(-rpm));
        } else {
            intake.setVelocity(0);
        }

        // Blocker servo control
        if (gamepad2.a) {
            blocker.setPosition(0.175);
        } else {
            blocker.setPosition(0.3);
        }

        // Shooter motor control
        if (gamepad2.left_bumper) {
            shooter.setPower(-0.8);
        } else if (gamepad2.left_trigger > 0.1) { // Added a deadzone for the trigger
            shooter.setPower(0.6);
        } else {
            shooter.setPower(0);
        }
    }

    /**
     * Handles the Mecanum drivetrain logic using gamepad 1.
     */
    private void handleDriveControls() {
        drive(gamepad1.left_stick_y, gamepad1.left_stick_x, gamepad1.right_stick_x);
    }

    /**
     * Converts RPM to ticks per second for the motor.
     */
    public double getTickSpeed(double speed) {
        return speed * TICKS_PER_REV / 60;
    }

    /**
     * Mecanum drive logic.
     * @param forward The forward/backward movement from the left stick y-axis.
     * @param right The strafing movement from the left stick x-axis.
     * @param rotate The rotational movement from the right stick x-axis.
     */
    public void drive(double forward, double right, double rotate) {
        // The signs for right and rotate might need to be inverted depending on motor setup
        double frontLeftPower = forward + right + rotate;
        double frontRightPower = forward - right - rotate;
        double backRightPower = forward + right - rotate;
        double backLeftPower = forward - right + rotate;

        // Find the maximum absolute power to maintain wheel ratios
        double maxPower = 1.0;
        maxPower = Math.max(maxPower, Math.abs(frontLeftPower));
        maxPower = Math.max(maxPower, Math.abs(frontRightPower));
        maxPower = Math.max(maxPower, Math.abs(backRightPower));
        maxPower = Math.max(maxPower, Math.abs(backLeftPower));

        // Normalize powers and apply a maximum speed limit
        double maxSpeed = 0.75;
        frontLeftDrive.setPower(maxSpeed * (frontLeftPower / maxPower));
        frontRightDrive.setPower(maxSpeed * (frontRightPower / maxPower));
        backLeftDrive.setPower(maxSpeed * (backLeftPower / maxPower));
        backRightDrive.setPower(maxSpeed * (backRightPower / maxPower));
    }

    /**
     * Displays all telemetry data to the Driver Station and FTC Dashboard.
     */
    private void displayTelemetry() {
        // Driver Controls and Subsystem Status
        telemetry.addLine("--- Driver & Subsystems ---");
        telemetry.addLine("G2 RT: Intake | G2 RB: Reverse Intake");
        telemetry.addData("Intake Target RPM", intake.getVelocity() / TICKS_PER_REV * 60);
        telemetry.addLine();

        // Limelight Controls and Status
        telemetry.addLine("--- Limelight ---");
        telemetry.addData("Camera View (Y on G1)", isNormalView ? "Normal" : "Tracking");
        telemetry.addData("LEDs (A on G1)", areLedsOn ? "On" : "Off");
        telemetry.addLine();

        // Robot Position (Odometry)
        telemetry.addLine("--- Odometry ---");
        telemetry.addData("X Position", follower.getPose().getX());
        telemetry.addData("Y Position", follower.getPose().getY());
        telemetry.addData("Odometry Heading (deg)", Math.toDegrees(follower.getPose().getHeading()));
        telemetry.addLine();

        // IMU Data
        telemetry.addLine("--- IMU ---");
        telemetry.addData("IMU Yaw (deg)", imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES));

        telemetry.update();
    }
}
