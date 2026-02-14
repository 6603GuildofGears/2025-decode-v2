package org.firstinspires.ftc.teamcode.pedroPathing.TeleOp.Test_codes;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.pedroPathing.Pipelines.BallDetectorPipeline;
import org.firstinspires.ftc.teamcode.pedroPathing.Pipelines.BallDetectorPipeline.BallDetection;

import java.util.List;

/**
 * Test OpMode for Ball Detection using YOLOv8n TFLite model.
 * 
 * This OpMode demonstrates how to:
 * 1. Initialize the ball detection pipeline
 * 2. Get detected balls sorted by ramp position (bottom first)
 * 3. Get the color sequence of balls
 * 4. Access individual ball properties
 * 
 * Make sure the TFLite model file (ftc_ball_nano_320.tflite) is in:
 * TeamCode/src/main/assets/
 */
@TeleOp(name = "Ball Detector Test", group = "Test")
public class BallDetectorTest extends LinearOpMode {

    @Override
    public void runOpMode() {
        
        // Initialize the ball detector (uses "Webcam 1" by default)
        telemetry.addData("Status", "Initializing Ball Detector...");
        telemetry.update();
        
        BallDetectorPipeline.init(this);
        
        // Wait for camera to be ready
        while (!isStarted() && !isStopRequested()) {
            if (BallDetectorPipeline.isReady()) {
                telemetry.addData("Status", "Camera Ready!");
            } else {
                telemetry.addData("Status", "Waiting for camera...");
            }
            
            // Show any early detections during init
            BallDetectorPipeline.pollOnce();
            displayDetections();
            
            telemetry.addData(">", "Press START to begin");
            telemetry.update();
            sleep(100);
        }
        
        waitForStart();
        
        while (opModeIsActive()) {
            // Poll for new detections
            BallDetectorPipeline.pollOnce();
            
            // Display all detection info
            displayDetections();
            
            // Example: React to specific ball patterns
            String sequence = BallDetectorPipeline.getBallSequence();
            
            // Example logic based on ball sequence
            if (sequence.startsWith("G")) {
                telemetry.addData("Action", "First ball is GREEN - ready for intake!");
            } else if (sequence.startsWith("P")) {
                telemetry.addData("Action", "First ball is PURPLE - ready for intake!");
            } else {
                telemetry.addData("Action", "No balls detected");
            }
            
            telemetry.update();
        }
        
        // Clean up
        BallDetectorPipeline.stop();
    }
    
    private void displayDetections() {
        telemetry.addData("=== BALL DETECTION ===", "");
        telemetry.addData("Ball Count", BallDetectorPipeline.getBallCount());
        telemetry.addData("Sequence (bottom→top)", BallDetectorPipeline.getBallSequence());
        
        // Display each ball
        List<BallDetection> balls = BallDetectorPipeline.getDetectedBalls();
        for (BallDetection ball : balls) {
            telemetry.addData("Ball #" + ball.rampPosition, 
                String.format("%s (%.0f%%) Y=%.0f", 
                    ball.isGreen ? "GREEN" : "PURPLE",
                    ball.confidence * 100,
                    ball.bottomY));
        }
        
        // Show counts by color
        telemetry.addData("Green Balls", BallDetectorPipeline.getGreenBalls().size());
        telemetry.addData("Purple Balls", BallDetectorPipeline.getPurpleBalls().size());
        
        // First ball info
        BallDetection first = BallDetectorPipeline.getFirstBall();
        if (first != null) {
            telemetry.addData("FIRST BALL", first.toString());
        }
    }
}
