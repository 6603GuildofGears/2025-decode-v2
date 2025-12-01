package org.firstinspires.ftc.teamcode;

// Imports for Dashboard
import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.IMU;

// Import the Pedro Pathing classes
import com.pedropathing.follower.Follower;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

// Limelight SDK imports
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.hardware.limelightvision.LLResult;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import java.util.List;
import java.io.File;
import java.io.FileWriter;
import java.io.BufferedReader;
import java.io.FileReader;
import java.io.IOException;
@Disabled
@Config
@TeleOp(name = "Localization Calibration", group = "Competition")
public class LocalizationCalibration extends OpMode {

    // Hardware variables
    private DcMotorEx frontLeftDrive, frontRightDrive, backLeftDrive, backRightDrive;
    private IMU imu;
    private Follower follower;
    private Limelight3A limelight;

    // Sensor fusion for position tracking
    private double fusedX = 0;
    private double fusedY = 0;
    private double lastOdoX = 0;
    private double lastOdoY = 0;
    public static double LIMELIGHT_WEIGHT = 0.85;
    
    // Field calibration offsets - TUNE THESE DURING FIELD INSPECTION
    public static double FIELD_OFFSET_X = 0.0;  // X offset correction (inches)
    public static double FIELD_OFFSET_Y = 0.0;  // Y offset correction (inches)
    
    // File path for saving calibration
    private static final String CALIBRATION_FILE = "/sdcard/FIRST/field_calibration.txt";
    private boolean savedThisSession = false;
    
    // AprilTag tracking
    private int currentTagId = -1;
    private String currentTagType = "None";
    private String currentTagLocation = "Unknown";
    
    // AprilTag field positions (Pedro Pathing coordinates - origin top-left, Y down, inches)
    // DECODE Season 2025-2026 - Field size: 141.24" x 141.24"
    private static final double[][] BLUE_TAG_POSITIONS = {
        {6, 47.25},    // Tag 20 - Blue Goal
        {6, 70.62},    // Tag 21 - Motif 21  
        {6, 93.99},    // Tag 22 - Motif 22
        {6, 117.36}    // Tag 23 - Motif 23
    };
    
    private static final String[] BLUE_TAG_NAMES = {
        "Blue Goal", "Motif 21", "Motif 22", "Motif 23"
    };
    
    private static final double[][] RED_TAG_POSITIONS = {
        {135.24, 47.25},   // Tag 24 - Red Goal
        {135.24, 70.62},   // Tag 25 - Motif 21
        {135.24, 93.99},   // Tag 26 - Motif 22
        {135.24, 117.36}   // Tag 27 - Motif 23
    };
    
    private static final String[] RED_TAG_NAMES = {
        "Red Goal", "Motif 21", "Motif 22", "Motif 23"
    };
    
    private static final double[][] NEUTRAL_TAG_POSITIONS = {
        {70.62, 6}, {47.25, 6}, {93.99, 6},
        {0, 0}, {0, 0}, {0, 0}, {0, 0}, {0, 0}, {0, 0}, {0, 0}
    };
    
    private static final String[] NEUTRAL_TAG_NAMES = {
        "Audience Center", "Audience Left", "Audience Right",
        "Neutral 4", "Neutral 5", "Neutral 6", "Neutral 7", 
        "Neutral 8", "Neutral 9", "Neutral 10"
    };

    @Override
    public void init() {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        // Hardware mapping
        frontLeftDrive = hardwareMap.get(DcMotorEx.class, "frontLeftDrive");
        frontRightDrive = hardwareMap.get(DcMotorEx.class, "frontRightDrive");
        backLeftDrive = hardwareMap.get(DcMotorEx.class, "backLeftDrive");
        backRightDrive = hardwareMap.get(DcMotorEx.class, "backRightDrive");
        imu = hardwareMap.get(IMU.class, "imu");
        
        follower = Constants.createFollower(hardwareMap);

        // Drivetrain setup
        frontLeftDrive.setDirection(DcMotorSimple.Direction.REVERSE);
        backLeftDrive.setDirection(DcMotorSimple.Direction.FORWARD);
        
        frontLeftDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        frontRightDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backLeftDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backRightDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

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
        
        // Load saved calibration values
        loadCalibration();

        telemetry.addLine("=== LOCALIZATION CALIBRATION ===");
        telemetry.addLine("For Competition Field Inspection");
        telemetry.addLine();
        telemetry.addLine("1. Drive to known field position");
        telemetry.addLine("2. Check Fused Position in telemetry");
        telemetry.addLine("3. Adjust offsets in FTC Dashboard");
        telemetry.addLine("4. Press Y to SAVE calibration");
        telemetry.addLine();
        telemetry.addData("Loaded Offset X", FIELD_OFFSET_X);
        telemetry.addData("Loaded Offset Y", FIELD_OFFSET_Y);
        telemetry.update();
    }

    @Override
    public void loop() {
        // Save calibration when Y button pressed
        if (gamepad1.y && !savedThisSession) {
            saveCalibration();
            savedThisSession = true;
        }
        
        // Simple drive controls
        double LStickY = -gamepad1.left_stick_x;
        double LStickX = -gamepad1.left_stick_y;
        double RStickX = gamepad1.right_stick_x;
        
        if (Math.abs(LStickX) > 0.1 || Math.abs(LStickY) > 0.1 || Math.abs(RStickX) > 0.1) {
            double rotation = 0;
            double newX = -LStickX * Math.cos(rotation) - -LStickY * Math.sin(rotation);
            double newY = LStickY * Math.cos(rotation) - -LStickX * Math.sin(rotation);
            double r = Math.hypot(newX, newY);
            double robotAngle = Math.atan2(newY, newX) - Math.PI / 4;

            double v1 = r * Math.cos(robotAngle) + RStickX;
            double v2 = r * Math.sin(robotAngle) + RStickX;
            double v3 = r * Math.sin(robotAngle) - RStickX;
            double v4 = r * Math.cos(robotAngle) - RStickX;

            frontLeftDrive.setPower(v1);
            frontRightDrive.setPower(v2);
            backLeftDrive.setPower(v3);
            backRightDrive.setPower(v4);
        } else {
            frontLeftDrive.setPower(0);
            frontRightDrive.setPower(0);
            backLeftDrive.setPower(0);
            backRightDrive.setPower(0);
        }

        // Update odometry and fused position
        follower.update();
        updateFusedPosition();
        displayTelemetry();
    }

    private void updateFusedPosition() {
        double odoX = follower.getPose().getX();
        double odoY = follower.getPose().getY();
        
        LLResult result = limelight.getLatestResult();
        if (result != null && result.isValid()) {
            List<LLResultTypes.FiducialResult> fiducials = result.getFiducialResults();
            if (!fiducials.isEmpty()) {
                LLResultTypes.FiducialResult tag = fiducials.get(0);
                int tagId = (int) tag.getFiducialId();
                currentTagId = tagId;
                
                double[] tagFieldPos = getTagFieldPosition(tagId);
                boolean isFixedTag = (tagId == 20 || tagId == 24 || (tagId >= 1 && tagId <= 10));
                
                if (tagFieldPos != null && isFixedTag) {
                    Pose3D robotPose = tag.getRobotPoseFieldSpace();
                    double llX = tagFieldPos[0] + (robotPose.getPosition().x * 39.3701) + FIELD_OFFSET_X;
                    double llY = tagFieldPos[1] - (robotPose.getPosition().y * 39.3701) + FIELD_OFFSET_Y;
                    
                    fusedX = (1.0 - LIMELIGHT_WEIGHT) * odoX + LIMELIGHT_WEIGHT * llX;
                    fusedY = (1.0 - LIMELIGHT_WEIGHT) * odoY + LIMELIGHT_WEIGHT * llY;
                    
                    lastOdoX = odoX;
                    lastOdoY = odoY;
                    return;
                }
            }
        }
        
        double deltaX = odoX - lastOdoX;
        double deltaY = odoY - lastOdoY;
        fusedX += deltaX;
        fusedY += deltaY;
        lastOdoX = odoX;
        lastOdoY = odoY;
    }
    
    private double[] getTagFieldPosition(int tagId) {
        if (tagId >= 1 && tagId <= 10) {
            currentTagType = "Neutral";
            currentTagLocation = NEUTRAL_TAG_NAMES[tagId - 1];
            return NEUTRAL_TAG_POSITIONS[tagId - 1];
        } else if (tagId >= 20 && tagId <= 23) {
            currentTagType = "Blue";
            currentTagLocation = BLUE_TAG_NAMES[tagId - 20];
            return BLUE_TAG_POSITIONS[tagId - 20];
        } else if (tagId >= 24 && tagId <= 27) {
            currentTagType = "Red";
            currentTagLocation = RED_TAG_NAMES[tagId - 24];
            return RED_TAG_POSITIONS[tagId - 24];
        }
        currentTagType = "Unknown";
        currentTagLocation = "Unknown";
        return null;
    }
    
    /**
     * Save calibration values to file on Robot Controller
     */
    private void saveCalibration() {
        try {
            File file = new File(CALIBRATION_FILE);
            file.getParentFile().mkdirs();
            FileWriter writer = new FileWriter(file);
            writer.write("FIELD_OFFSET_X=" + FIELD_OFFSET_X + "\n");
            writer.write("FIELD_OFFSET_Y=" + FIELD_OFFSET_Y + "\n");
            writer.write("LIMELIGHT_WEIGHT=" + LIMELIGHT_WEIGHT + "\n");
            writer.close();
            
            telemetry.speak("Calibration saved");
        } catch (IOException e) {
            telemetry.addData("Save Error", e.getMessage());
        }
    }
    
    /**
     * Load calibration values from file on Robot Controller
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

    private void displayTelemetry() {
        telemetry.addLine("=== CALIBRATION MODE ===");
        telemetry.addLine();
        
        // AprilTag Detection
        telemetry.addLine("--- AprilTag ---");
        if (currentTagId != -1) {
            telemetry.addData("Tag ID", currentTagId);
            telemetry.addData("Type", currentTagType);
            telemetry.addData("Location", currentTagLocation);
        } else {
            telemetry.addData("Tag", "None Detected");
        }
        telemetry.addLine();
        
        // Fused Position
        telemetry.addLine("--- FUSED POSITION ---");
        telemetry.addData("X", "%.2f in", fusedX);
        telemetry.addData("Y", "%.2f in", fusedY);
        telemetry.addData("Heading", "%.1f°", Math.toDegrees(follower.getPose().getHeading()));
        telemetry.addLine();
        
        // Raw Odometry
        telemetry.addLine("--- Raw Odometry ---");
        telemetry.addData("Odo X", "%.2f", follower.getPose().getX());
        telemetry.addData("Odo Y", "%.2f", follower.getPose().getY());
        telemetry.addLine();
        
        // Calibration Settings
        telemetry.addLine("--- CALIBRATION (FTC Dashboard) ---");
        telemetry.addData("Offset X", "%.2f", FIELD_OFFSET_X);
        telemetry.addData("Offset Y", "%.2f", FIELD_OFFSET_Y);
        telemetry.addData("LL Weight", "%.2f", LIMELIGHT_WEIGHT);
        telemetry.addLine();
        
        telemetry.addLine("INSTRUCTIONS:");
        telemetry.addLine("• Drive to field corner/tile edge");
        telemetry.addLine("• Note expected X,Y position");
        telemetry.addLine("• Adjust offsets in Dashboard");
        telemetry.addLine("• Press Y to SAVE to Robot");
        telemetry.addLine();
        
        if (savedThisSession) {
            telemetry.addLine("✓ CALIBRATION SAVED!");
        } else {
            telemetry.addLine("Press Y to save calibration");
        }

        telemetry.update();
    }
}
