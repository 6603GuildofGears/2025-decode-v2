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
import java.io.File;
import java.io.BufferedReader;
import java.io.FileReader;
import java.io.IOException;

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
    
    // Sensor fusion for position tracking
    private double fusedX = 0;
    private double fusedY = 0;
    private double lastOdoX = 0;  // Last odometry reading
    private double lastOdoY = 0;
    private double lastLimelightUpdateTime = 0;
    public static double LIMELIGHT_WEIGHT = 0.85;  // Weight for Limelight updates (tunable via dashboard)
    
    // Field calibration offsets (loaded from file saved during field inspection)
    public static double FIELD_OFFSET_X = 0.0;
    public static double FIELD_OFFSET_Y = 0.0;
    private static final String CALIBRATION_FILE = "/sdcard/FIRST/field_calibration.txt";
    
    // AprilTag tracking
    private int currentTagId = -1;
    private String currentTagType = "None";
    private String currentTagLocation = "Unknown";
    
    // AprilTag field positions (Pedro Pathing coordinates - origin top-left, Y down, inches)
    // DECODE Season 2025-2026 - Field size: 141.24" x 141.24"
    // Blue alliance tags (IDs 20, 21, 22, 23)
    private static final double[][] BLUE_TAG_POSITIONS = {
        {6, 47.25},    // Tag 20 - Blue Goal
        {6, 70.62},    // Tag 21 - Motif 21  
        {6, 93.99},    // Tag 22 - Motif 22
        {6, 117.36}    // Tag 23 - Motif 23
    };
    
    private static final String[] BLUE_TAG_NAMES = {
        "Blue Goal",   // Tag 20
        "Motif 21",    // Tag 21
        "Motif 22",    // Tag 22
        "Motif 23"     // Tag 23
    };
    
    // Red alliance tags (IDs 24, 25, 26, 27)
    private static final double[][] RED_TAG_POSITIONS = {
        {135.24, 47.25},   // Tag 24 - Red Goal
        {135.24, 70.62},   // Tag 25 - Motif 21
        {135.24, 93.99},   // Tag 26 - Motif 22
        {135.24, 117.36}   // Tag 27 - Motif 23
    };
    
    private static final String[] RED_TAG_NAMES = {
        "Red Goal",    // Tag 24
        "Motif 21",    // Tag 25
        "Motif 22",    // Tag 26
        "Motif 23"     // Tag 27
    };
    
    // Neutral tags (IDs 1-10)
    private static final double[][] NEUTRAL_TAG_POSITIONS = {
        {70.62, 6},       // Tag 1 - Audience wall center
        {47.25, 6},       // Tag 2 - Audience wall left
        {93.99, 6},       // Tag 3 - Audience wall right
        {0, 0},           // Tag 4-10 (adjust as needed)
        {0, 0},
        {0, 0},
        {0, 0},
        {0, 0},
        {0, 0},
        {0, 0}
    };
    
    private static final String[] NEUTRAL_TAG_NAMES = {
        "Audience Center",  // Tag 1
        "Audience Left",    // Tag 2
        "Audience Right",   // Tag 3
        "Neutral 4",        // Tags 4-10
        "Neutral 5",
        "Neutral 6",
        "Neutral 7",
        "Neutral 8",
        "Neutral 9",
        "Neutral 10"
    };

    @Override
    public void init() {
        // Set up FTC Dashboard
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        
        // Load field calibration from saved file
        loadCalibration();

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
        
        // Update fused position using Limelight + Odometry
        updateFusedPosition();

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
                int tagId = (int) tag.getFiducialId();
                
                // Only auto-aim to goal tags (20 = Blue Goal, 24 = Red Goal)
                // Ignore motif tags (21-23, 25-27)
                if (tagId != 20 && tagId != 24) {
                    return 0;  // Don't auto-aim to motif tags
                }
                
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
     * Update fused position using Limelight absolute position and odometry relative tracking
     */
    private void updateFusedPosition() {
        // Get current odometry position
        double odoX = follower.getPose().getX();
        double odoY = follower.getPose().getY();
        
        // Get Limelight data
        LLResult result = limelight.getLatestResult();
        if (result != null && result.isValid()) {
            List<LLResultTypes.FiducialResult> fiducials = result.getFiducialResults();
            if (!fiducials.isEmpty()) {
                LLResultTypes.FiducialResult tag = fiducials.get(0);
                int tagId = (int) tag.getFiducialId();
                currentTagId = tagId;
                
                // Determine tag type and get field position
                double[] tagFieldPos = getTagFieldPosition(tagId);
                
                // Only use fixed goal tags (20, 24) for localization, not motif tags (21-23, 25-27)
                boolean isFixedTag = (tagId == 20 || tagId == 24 || (tagId >= 1 && tagId <= 10));
                
                if (tagFieldPos != null && isFixedTag) {
                    // Get robot pose from Limelight (relative to detected tag)
                    Pose3D robotPose = tag.getRobotPoseFieldSpace();
                    
                    // Calculate robot position from tag position
                    // Convert to Pedro Pathing coordinates (inches, origin top-left)
                    double llX = tagFieldPos[0] + (robotPose.getPosition().x * 39.3701) + FIELD_OFFSET_X;
                    double llY = tagFieldPos[1] - (robotPose.getPosition().y * 39.3701) + FIELD_OFFSET_Y;
                    
                    // Blend positions: high Limelight weight for accuracy
                    fusedX = (1.0 - LIMELIGHT_WEIGHT) * odoX + LIMELIGHT_WEIGHT * llX;
                    fusedY = (1.0 - LIMELIGHT_WEIGHT) * odoY + LIMELIGHT_WEIGHT * llY;
                    
                    // Store current odometry for delta tracking
                    lastOdoX = odoX;
                    lastOdoY = odoY;
                    lastLimelightUpdateTime = System.currentTimeMillis();
                    return;
                }
            }
        }
        
        // No valid Limelight data: add odometry delta to last fused position (prevents jump)
        // Don't reset currentTagId/Type immediately - keep last known tag info
        double deltaX = odoX - lastOdoX;
        double deltaY = odoY - lastOdoY;
        fusedX += deltaX;
        fusedY += deltaY;
        
        // Update odometry reference
        lastOdoX = odoX;
        lastOdoY = odoY;
    }
    
    /**
     * Get the field position of an AprilTag by ID and set tag type
     */
    private double[] getTagFieldPosition(int tagId) {
        // Neutral tags (1-10)
        if (tagId >= 1 && tagId <= 10) {
            currentTagType = "Neutral";
            currentTagLocation = NEUTRAL_TAG_NAMES[tagId - 1];
            return NEUTRAL_TAG_POSITIONS[tagId - 1];
        }
        // Blue alliance tags (20-23)
        else if (tagId >= 20 && tagId <= 23) {
            currentTagType = "Blue";
            currentTagLocation = BLUE_TAG_NAMES[tagId - 20];
            return BLUE_TAG_POSITIONS[tagId - 20];
        }
        // Red alliance tags (24-27)
        else if (tagId >= 24 && tagId <= 27) {
            currentTagType = "Red";
            currentTagLocation = RED_TAG_NAMES[tagId - 24];
            return RED_TAG_POSITIONS[tagId - 24];
        }
        
        currentTagType = "Unknown";
        currentTagLocation = "Unknown";
        return null;
    }
    
    /**
     * Load calibration values from file saved during field inspection
     */
    private void loadCalibration() {
        try {
            File file = new File(CALIBRATION_FILE);
            if (file.exists()) {
                BufferedReader reader = new BufferedReader(new FileReader(file));
                String line;
                while ((line = reader.readLine()) != null) {
                    if (line.startsWith("FIELD_OFFSET_X=")) {
                        FIELD_OFFSET_X = Double.parseDouble(line.substring(15));
                    } else if (line.startsWith("FIELD_OFFSET_Y=")) {
                        FIELD_OFFSET_Y = Double.parseDouble(line.substring(15));
                    } else if (line.startsWith("LIMELIGHT_WEIGHT=")) {
                        LIMELIGHT_WEIGHT = Double.parseDouble(line.substring(17));
                    }
                }
                reader.close();
            }
        } catch (IOException e) {
            // No saved calibration, use defaults
        }
    }

    /**
     * Displays all telemetry data to the Driver Station and FTC Dashboard.
     */
    private void displayTelemetry() {
        // Auto-Aim Status
        telemetry.addLine("--- Auto-Aim ---");
        telemetry.addData("Auto-Aim", autoAimEnabled ? "ENABLED" : "DISABLED");
        telemetry.addLine();
        
        // AprilTag Detection
        telemetry.addLine("--- AprilTag Detection ---");
        if (currentTagId != -1) {
            telemetry.addData("Tag ID", currentTagId);
            telemetry.addData("Tag Type", currentTagType);
            telemetry.addData("Location", currentTagLocation);
        } else {
            telemetry.addData("Tag", "None Detected");
        }
        telemetry.addLine();
        
        // Limelight Status
        telemetry.addLine("--- Limelight ---");
        LLResult result = limelight.getLatestResult();
        if (result != null && result.isValid()) {
            telemetry.addData("LL Latency", "%.1f ms", result.getCaptureLatency() + result.getTargetingLatency());
        } else {
            telemetry.addData("Limelight", "No data available");
        }
        telemetry.addLine();

        // Robot Position (Fused: Limelight + Odometry)
        telemetry.addLine("--- Fused Position ---");
        telemetry.addData("X Position", "%.2f", fusedX);
        telemetry.addData("Y Position", "%.2f", fusedY);
        telemetry.addData("Heading (deg)", "%.1f", Math.toDegrees(follower.getPose().getHeading()));
        telemetry.addData("LL Weight", "%.2f", LIMELIGHT_WEIGHT);

        telemetry.update();
    }
}
