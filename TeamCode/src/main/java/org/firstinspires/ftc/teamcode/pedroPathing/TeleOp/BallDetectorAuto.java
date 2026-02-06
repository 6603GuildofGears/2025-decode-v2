package org.firstinspires.ftc.teamcode.pedroPathing.TeleOp;

import android.graphics.Bitmap;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.opencv.android.Utils;
import org.opencv.core.Mat;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvWebcam;

/**
 * Ball Detector Autonomous Mode
 * 
 * Uses "da-ball-cam" webcam to detect purple and green wiffle balls.
 * Displays ball count and colors in order on the REV Control Hub screen.
 * Robot can be controlled via gamepad while detecting.
 * 
 * Camera stream with detection overlay is sent to FTC Dashboard/Panels.
 * 
 * Controls:
 * - Left stick: Drive (forward/backward/strafe)
 * - Right stick X: Rotate
 * - Left/Right bumpers: Strafe left/right
 */
@TeleOp(name = "Ball Detector", group = "Vision")
public class BallDetectorAuto extends LinearOpMode {
    
    // Camera and pipeline
    private OpenCvWebcam webcam;
    private WiffleBallPipeline pipeline;
    
    // FTC Dashboard for camera streaming
    private FtcDashboard dashboard;
    
    // Drive motors
    private DcMotor frontLeft;
    private DcMotor frontRight;
    private DcMotor backLeft;
    private DcMotor backRight;
    
    // Drive speed modifier
    private double driveSpeed = 0.7;
    
    @Override
    public void runOpMode() {
        // Initialize FTC Dashboard (used by Panels for camera streaming)
        dashboard = FtcDashboard.getInstance();
        dashboard.setTelemetryTransmissionInterval(25); // Faster updates for smoother stream
        
        // Initialize camera
        initCamera();
        
        // Initialize drive motors
        initDriveMotors();
        
        // Wait for camera to initialize
        telemetry.addLine("=== BALL DETECTOR ===");
        telemetry.addLine("Camera: da-ball-cam");
        telemetry.addLine("");
        telemetry.addLine("Camera stream available at:");
        telemetry.addLine("  Panels: http://192.168.43.1:8001");
        telemetry.addLine("  Dashboard: http://192.168.43.1:8080/dash");
        telemetry.addLine("");
        telemetry.addLine("In Panels: Click 'Camera' tab and enable stream");
        telemetry.update();
        
        // Show detection while waiting for start
        while (!isStarted() && !isStopRequested()) {
            displayBallTelemetry();
            sleep(100);
        }
        
        // Main loop - detect balls and allow gamepad control
        while (opModeIsActive()) {
            // Handle driving
            handleDriving();
            
            // Display ball detection results
            displayBallTelemetry();
            
            sleep(20); // Small delay to prevent overwhelming the system
        }
        
        // Cleanup
        if (webcam != null) {
            dashboard.stopCameraStream();
            webcam.stopStreaming();
        }
    }
    
    /**
     * Initialize the webcam and vision pipeline
     */
    private void initCamera() {
        // Get camera monitor view ID for live preview on Driver Station
        int cameraMonitorViewId = hardwareMap.appContext.getResources()
            .getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        
        // Create webcam instance using "da-ball-cam"
        webcam = OpenCvCameraFactory.getInstance()
            .createWebcam(hardwareMap.get(WebcamName.class, "da-ball-cam"), cameraMonitorViewId);
        
        // Create and set the ball detection pipeline
        pipeline = new WiffleBallPipeline();
        webcam.setPipeline(pipeline);
        
        // Set viewport renderer to GPU accelerated for better streaming
        webcam.setViewportRenderingPolicy(OpenCvCamera.ViewportRenderingPolicy.OPTIMIZE_VIEW);
        webcam.setViewportRenderer(OpenCvCamera.ViewportRenderer.GPU_ACCELERATED);
        
        // Open camera and start streaming
        webcam.setMillisecondsPermissionTimeout(2500);
        webcam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {
                // Start streaming at 640x480
                webcam.startStreaming(640, 480, OpenCvCameraRotation.UPRIGHT);
                
                // Start streaming to FTC Dashboard/Panels with detection overlay
                // Use max FPS of 0 to let dashboard control the frame rate
                FtcDashboard.getInstance().startCameraStream(webcam, 0);
            }
            
            @Override
            public void onError(int errorCode) {
                telemetry.addData("Camera Error", errorCode);
                telemetry.update();
            }
        });
    }
    
    /**
     * Initialize drive motors
     */
    private void initDriveMotors() {
        try {
            frontLeft = hardwareMap.get(DcMotor.class, "frontLeft");
            frontRight = hardwareMap.get(DcMotor.class, "frontRight");
            backLeft = hardwareMap.get(DcMotor.class, "backLeft");
            backRight = hardwareMap.get(DcMotor.class, "backRight");
            
            // Set motor directions (adjust based on your robot configuration)
            frontLeft.setDirection(DcMotorSimple.Direction.REVERSE);
            backLeft.setDirection(DcMotorSimple.Direction.FORWARD);
            frontRight.setDirection(DcMotorSimple.Direction.FORWARD);
            backRight.setDirection(DcMotorSimple.Direction.FORWARD);
            
            // Set zero power behavior
            frontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            frontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            backLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            backRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            
            telemetry.addLine("Drive motors initialized");
        } catch (Exception e) {
            telemetry.addLine("WARNING: Drive motors not found!");
            telemetry.addLine("Robot will not move, but detection will work.");
        }
    }
    
    /**
     * Handle gamepad input for driving
     */
    private void handleDriving() {
        if (frontLeft == null || frontRight == null || backLeft == null || backRight == null) {
            return; // Motors not initialized
        }
        
        // Get gamepad inputs
        double leftStickY = -gamepad1.left_stick_y;  // Forward/backward
        double leftStickX = gamepad1.left_stick_x;   // Strafe
        double rightStickX = gamepad1.right_stick_x; // Rotation
        
        // Bumper strafing
        if (gamepad1.left_bumper) {
            leftStickX = -0.5;
        } else if (gamepad1.right_bumper) {
            leftStickX = 0.5;
        }
        
        // Speed control with triggers
        if (gamepad1.left_trigger > 0.1) {
            driveSpeed = 0.4; // Slow mode
        } else if (gamepad1.right_trigger > 0.1) {
            driveSpeed = 1.0; // Fast mode
        } else {
            driveSpeed = 0.7; // Normal mode
        }
        
        // Calculate mecanum drive powers
        double r = Math.hypot(leftStickX, leftStickY);
        double robotAngle = Math.atan2(leftStickY, leftStickX) - Math.PI / 4;
        
        double v1 = r * Math.cos(robotAngle) + rightStickX; // Front left
        double v2 = r * Math.sin(robotAngle) - rightStickX; // Front right
        double v3 = r * Math.sin(robotAngle) + rightStickX; // Back left
        double v4 = r * Math.cos(robotAngle) - rightStickX; // Back right
        
        // Normalize if any value exceeds 1.0
        double max = Math.max(Math.max(Math.abs(v1), Math.abs(v2)), 
                              Math.max(Math.abs(v3), Math.abs(v4)));
        if (max > 1.0) {
            v1 /= max;
            v2 /= max;
            v3 /= max;
            v4 /= max;
        }
        
        // Apply speed modifier and set motor powers
        frontLeft.setPower(v1 * driveSpeed);
        frontRight.setPower(v2 * driveSpeed);
        backLeft.setPower(v3 * driveSpeed);
        backRight.setPower(v4 * driveSpeed);
    }
    
    /**
     * Display ball detection results on telemetry (Control Hub screen)
     */
    private void displayBallTelemetry() {
        telemetry.addLine("╔══════════════════════════════╗");
        telemetry.addLine("║    🎱 BALL DETECTOR 🎱       ║");
        telemetry.addLine("╠══════════════════════════════╣");
        
        // Ball count
        int ballCount = pipeline.getBallCount();
        int purpleCount = pipeline.getPurpleCount();
        int greenCount = pipeline.getGreenCount();
        
        telemetry.addData("║ Total Balls", ballCount + "                  ║");
        telemetry.addData("║ Purple", purpleCount + " | Green: " + greenCount + "          ║");
        
        telemetry.addLine("╠══════════════════════════════╣");
        telemetry.addLine("║       COLORS IN ORDER        ║");
        telemetry.addLine("╠══════════════════════════════╣");
        
        // Display each ball's color in order
        if (ballCount > 0) {
            String[] pattern = pipeline.getPattern();
            StringBuilder orderDisplay = new StringBuilder();
            
            for (int i = 0; i < ballCount && i < 9; i++) {
                String color = pattern[i];
                if (color.equals("P")) {
                    orderDisplay.append("🟣"); // Purple
                } else if (color.equals("G")) {
                    orderDisplay.append("🟢"); // Green
                } else {
                    orderDisplay.append("⚪"); // Unknown
                }
                if (i < ballCount - 1 && i < 8) {
                    orderDisplay.append(" → ");
                }
            }
            telemetry.addLine("║ " + orderDisplay.toString());
            
            // Also show as text
            telemetry.addData("║ Pattern", pipeline.getPatternString() + "           ║");
            
            // Detailed list
            telemetry.addLine("╠══════════════════════════════╣");
            telemetry.addLine("║        BALL DETAILS          ║");
            for (int i = 1; i <= ballCount && i <= 9; i++) {
                String colorSymbol = pipeline.getBallColor(i);
                String colorName = colorSymbol.equals("P") ? "Purple" : 
                                   colorSymbol.equals("G") ? "Green" : "Unknown";
                telemetry.addData("║ Ball " + i, colorName + "              ║");
            }
        } else {
            telemetry.addLine("║     No balls detected        ║");
        }
        
        telemetry.addLine("╠══════════════════════════════╣");
        telemetry.addLine("║         CONTROLS             ║");
        telemetry.addLine("║ L-Stick: Drive  R-Stick: Turn║");
        telemetry.addLine("║ LT: Slow  RT: Fast           ║");
        telemetry.addLine("╚══════════════════════════════╝");
        
        telemetry.update();
        
        // ===== Also send to FTC Dashboard =====
        TelemetryPacket packet = new TelemetryPacket();
        packet.put("Total Balls", ballCount);
        packet.put("Purple Count", purpleCount);
        packet.put("Green Count", greenCount);
        packet.put("Pattern", pipeline.getPatternString());
        for (int i = 1; i <= 9; i++) {
            String colorSymbol = pipeline.getBallColor(i);
            String colorName = colorSymbol.equals("P") ? "Purple" : 
                               colorSymbol.equals("G") ? "Green" : 
                               colorSymbol.equals("_") ? "Empty" : "Unknown";
            packet.put("Ball " + i, colorName);
        }
        dashboard.sendTelemetryPacket(packet);
    }
}
