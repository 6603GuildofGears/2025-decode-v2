package org.firstinspires.ftc.teamcode;

// Imports for Dashboard
import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

// Import the Pedro Pathing classes
import com.pedropathing.follower.Follower;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

// Limelight SDK imports
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLStatus;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import java.util.List;

@Config
@TeleOp(name = "Rango", group = "Robot")
public class Rango extends OpMode {

    // Hardware variables - Use DcMotorEx for all motors for consistency
    private DcMotorEx frontLeftDrive, frontRightDrive, backLeftDrive, backRightDrive;
    private DcMotorEx intake;
    private DcMotorEx shooter;
    private Servo blocker;
    private IMU imu;

    // Pedro Pathing Follower
    private Follower follower;

    // Limelight Variables
    private Limelight3A limelight;

    // Constants
    private final double TICKS_PER_REV = 28;
    private final double gear = 1.0;  // Gear ratio for rotation speed
    
    // Auto-aim constants (tunable via FTC Dashboard)
    public static double KP_ROTATE = 0.02;
    public static double ROTATE_DEADBAND = 3.0;
    public static double MAX_AUTO_ROTATE_SPEED = 0.5;
    
    // Auto-aim state
    private boolean autoAimEnabled = true;  // Start enabled
    private boolean lastAButtonState = false;

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
        follower = Constants.createFollower(hardwareMap);

        // Drivetrain setup
        frontLeftDrive.setDirection(DcMotorSimple.Direction.REVERSE);
        backLeftDrive.setDirection(DcMotorSimple.Direction.FORWARD);

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
        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        telemetry.setMsTransmissionInterval(11);
        limelight.pipelineSwitch(0);
        limelight.start();

        telemetry.addLine("Initialization Complete");
        telemetry.update();
    }

    @Override
    public void start() {
        blocker.setPosition(0.3);
    }

    @Override
    public void loop() {
        // Gamepad input variables
        boolean LStickIn2 = gamepad2.left_stick_button;
        boolean RStickIn2 = gamepad2.right_stick_button;
        boolean LBumper1 = gamepad1.left_bumper;
        boolean RBumper1 = gamepad1.right_bumper;

        double LStickY = gamepad1.left_stick_y;
        double LStickX = gamepad1.left_stick_x;
        double RStickY = -gamepad1.right_stick_y;
        double RStickX = -gamepad1.right_stick_x;

        double LTrigger1 = gamepad1.left_trigger;
        double RTrigger1 = gamepad1.right_trigger;

        boolean a1 = gamepad1.a;
        boolean b1 = gamepad1.b;
        boolean x1 = gamepad1.x;
        boolean y1 = gamepad1.y;

        boolean a2 = gamepad2.a;
        boolean b2 = gamepad2.b;
        boolean x2 = gamepad2.x;
        boolean y2 = gamepad2.y;

        double LTrigger2 = gamepad2.left_trigger;
        double RTrigger2 = gamepad2.right_trigger;
        boolean LBumper2 = gamepad2.left_bumper;
        boolean RBumper2 = gamepad2.right_bumper;

        double RStickY2 = -gamepad2.right_stick_y;
        double RStickX2 = gamepad2.right_stick_x;
        double LStickY2 = -gamepad2.left_stick_y;
        double LStickX2 = gamepad2.left_stick_x;

        boolean dpadUp1 = gamepad1.dpad_up;
        boolean dpadDown1 = gamepad1.dpad_down;
        boolean dpadRight1 = gamepad1.dpad_right;
        boolean dpadLeft1 = gamepad1.dpad_left;

        boolean dpadUp2 = gamepad2.dpad_up;
        boolean dpadDown2 = gamepad2.dpad_down;
        boolean dpadRight2 = gamepad2.dpad_right;
        boolean dpadLeft2 = gamepad2.dpad_left;

        // Toggle auto-aim with gamepad1 A button
        if (a1 && !lastAButtonState) {
            autoAimEnabled = !autoAimEnabled;
        }
        lastAButtonState = a1;

        // Update odometry localization
        follower.update();

        // Handle all robot controls
        handleSubsystemControls();
        handleDriveControls();

        // Display telemetry
        displayTelemetry();
    }

    /**
     * Handles the logic for the intake, shooter, and blocker mechanisms using gamepad 2.
     */
    private void handleSubsystemControls() {
        // Intake motor control (simplified and corrected)
        double rpm = 3500;
        if (gamepad2.right_bumper) {
            // intake.setVelocity(getTickSpeed(rpm));
            intake.setPower(1.0);
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
        double LStickY = -gamepad1.left_stick_x;  // Reversed forward/back
        double LStickX = -gamepad1.left_stick_y;  // Reversed strafe
        double RStickX = gamepad1.right_stick_x;  // Reversed turn
        
        boolean LBumper1 = gamepad1.left_bumper;
        boolean RBumper1 = gamepad1.right_bumper;
        
        boolean dpadUp1 = gamepad1.dpad_up;
        boolean dpadDown1 = gamepad1.dpad_down;
        boolean dpadRight1 = gamepad1.dpad_right;
        boolean dpadLeft1 = gamepad1.dpad_left;
        
        double gear = 1;

        if (Math.abs(LStickX) > 0 || Math.abs(LStickY) > 0 || Math.abs(RStickX) > 0) {
            double rotation = 0;

            double newX = -LStickX * Math.cos(rotation) - -LStickY * Math.sin(rotation);
            double newY = LStickY * Math.cos(rotation) - -LStickX * Math.sin(rotation);

            double r = Math.hypot(newX, newY);
            double robotAngle = Math.atan2(newY, newX) - Math.PI / 4;
            double rightX = RStickX;  // Use the stick value
            
            // Auto-aim override: rotate to face AprilTag if enabled and right stick centered
            if (autoAimEnabled && Math.abs(RStickX) < 0.1) {
                rightX = getAutoAimRotation();
            }

            double v1 = r * Math.cos(robotAngle) + rightX * gear; //lf
            double v2 = r * Math.sin(robotAngle) + rightX * gear; //rf
            double v3 = r * Math.sin(robotAngle) - rightX * gear; //lb
            double v4 = r * Math.cos(robotAngle) - rightX * gear; //rb

            SetPower(v1, v2, v3, v4);

        } else if (LBumper1) {
            SetPower(-gear, gear, gear, -gear);

        } else if (RBumper1) {
            SetPower(gear, -gear, -gear, gear);

        } else if (dpadUp1) {
            SetPower(1, 1, 1, 1);
        } else if (dpadRight1) {
            SetPower(1, -1, -1, 1);
        } else if (dpadLeft1) {
            SetPower(-1, 1, 1, -1);
        } else if (dpadDown1) {
            SetPower(-1, -1, -1, -1);

        } else {
            // Auto-aim when joysticks are released - apply auto-rotation
            if (autoAimEnabled) {
                double autoRotate = getAutoAimRotation();
                if (Math.abs(autoRotate) > 0.01) {
                    // Apply pure rotation to all wheels
                    SetPower(autoRotate * gear, autoRotate * gear, -autoRotate * gear, -autoRotate * gear);
                    return;
                }
            }
            
            frontLeftDrive.setPower(0);
            backLeftDrive.setPower(0);
            frontRightDrive.setPower(0);
            backRightDrive.setPower(0);
        }
    }

    /**
     * Converts RPM to ticks per second for the motor.
     */
    public double getTickSpeed(double speed) {
        return speed * TICKS_PER_REV / 60;
    }

    /**
     * Set power to all drive motors
     */
    private void SetPower(double v1, double v2, double v3, double v4) {
        frontLeftDrive.setPower(v1);
        frontRightDrive.setPower(v2);
        backLeftDrive.setPower(v3);
        backRightDrive.setPower(v4);
    }

    /**
     * Calculate auto-aim rotation to face AprilTag
     */
    private double getAutoAimRotation() {
        if (limelight == null) {
            return 0;
        }
        
        LLResult result = limelight.getLatestResult();
        if (result != null && result.isValid()) {
            List<LLResultTypes.FiducialResult> fiducials = result.getFiducialResults();
            if (!fiducials.isEmpty()) {
                LLResultTypes.FiducialResult tag = fiducials.get(0);
                double tx = tag.getTargetXDegrees();
                
                if (Math.abs(tx) > ROTATE_DEADBAND) {
                    double rotation = tx * KP_ROTATE;  // Positive to turn toward tag
                    return clamp(rotation, -MAX_AUTO_ROTATE_SPEED, MAX_AUTO_ROTATE_SPEED);
                }
            }
        }
        return 0;
    }

    /**
     * Clamp value between min and max
     */
    private double clamp(double value, double min, double max) {
        return Math.max(min, Math.min(max, value));
    }

    /**
     * Mecanum drive logic.
     * @param forward The forward/backward movement from the left stick y-axis.
     * @param strafe The strafing movement from the left stick x-axis.
     * @param rotate The rotational movement from the right stick x-axis.
     */
    public void drive(double forward, double strafe, double rotate) {
        // Correct mecanum drive formula
        double frontLeftPower = forward + strafe + rotate;
        double frontRightPower = forward - strafe - rotate;
        double backLeftPower = forward - strafe + rotate;
        double backRightPower = forward + strafe - rotate;

        // Normalize powers to prevent exceeding 1.0
        double maxPower = Math.max(1.0, Math.abs(frontLeftPower));
        maxPower = Math.max(maxPower, Math.abs(frontRightPower));
        maxPower = Math.max(maxPower, Math.abs(backLeftPower));
        maxPower = Math.max(maxPower, Math.abs(backRightPower));

        // Apply normalized powers
        frontLeftDrive.setPower(frontLeftPower / maxPower);
        frontRightDrive.setPower(frontRightPower / maxPower);
        backLeftDrive.setPower(backLeftPower / maxPower);
        backRightDrive.setPower(backRightPower / maxPower);
    }

    /**
     * Displays all telemetry data to the Driver Station and FTC Dashboard.
     */
    private void displayTelemetry() {
        // Auto-Aim Status
        telemetry.addLine("--- Auto-Aim ---");
        telemetry.addData("Auto-Aim", autoAimEnabled ? "ENABLED" : "DISABLED");
        telemetry.addLine();
        
        // Driver Controls and Subsystem Status
        telemetry.addLine("--- Driver & Subsystems ---");
        telemetry.addLine("G2 RT: Intake | G2 RB: Reverse Intake");
        telemetry.addData("Intake Target RPM", intake.getVelocity() / TICKS_PER_REV * 60);
        telemetry.addLine();

        // Limelight Field Coordinates
        telemetry.addLine("--- Limelight ---");
        LLResult result = limelight.getLatestResult();
        if (result != null && result.isValid()) {
            Pose3D botpose = result.getBotpose();
            telemetry.addData("Field X", "%.2f", (botpose.getPosition().x * (39+(47/127))));
            telemetry.addData("Field Y", "%.2f", (botpose.getPosition().y * (39+(47/127))));
            telemetry.addData("Field Z", "%.2f", (botpose.getPosition().z * (39+(47/127))));
            telemetry.addData("LL Latency", "%.1f ms", result.getCaptureLatency() + result.getTargetingLatency());
        } else {
            telemetry.addData("Limelight", "No data available");
        }
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
