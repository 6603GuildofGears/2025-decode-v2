package org.firstinspires.ftc.teamcode.pedroPathing.Autos.Motif;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.pedropathing.util.Timer;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.*;
import com.pedropathing.paths.*;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;

import org.firstinspires.ftc.teamcode.pedroPathing.other.Constants;
import org.firstinspires.ftc.teamcode.pedroPathing.Pipelines.Limelight_Pipeline;
import org.firstinspires.ftc.teamcode.pedroPathing.Pipelines.Motor_PipeLine;
import org.firstinspires.ftc.teamcode.pedroPathing.Pipelines.Motif;
import org.firstinspires.ftc.teamcode.pedroPathing.Pipelines.MotifQueue;
import org.firstinspires.ftc.teamcode.pedroPathing.Pipelines.SpindexerController;
import org.firstinspires.ftc.teamcode.pedroPathing.Pipelines.Servo_Pipeline;
import org.firstinspires.ftc.teamcode.pedroPathing.Pipelines.Sensor;
import org.firstinspires.ftc.teamcode.pedroPathing.TeleOp.ShooterLookup;
import org.firstinspires.ftc.teamcode.pedroPathing.TeleOp.Test_codes.BallDetectorPipeline;
import org.firstinspires.ftc.teamcode.pedroPathing.TeleOp.Test_codes.BallDetectorPipeline.BallDetection;

import java.util.List;

import static org.firstinspires.ftc.teamcode.pedroPathing.TeleOp.TurretConfig.*;

/**
 * Blue Front Auto with Motif detection.
 * Scans for obelisk tag (21/22/23) during init_loop,
 * then shoots balls in the motif-required color order.
 */
@Autonomous(name = "Blue Front MOTIF", group = "Blue")
public class Blue_Front_Motif extends OpMode {
    private Follower follower;
    private Timer pathTimer, opmodeTimer;
    private SpindexerController spindexer;
    private DcMotorEx flywheel;
    private DcMotorEx intake;
    private DcMotorEx intakeMotor2;
    private DcMotorEx turret;
    private Motor_PipeLine motorPipeline;
    private Servo_Pipeline servoPipeline;
    private boolean intakeRunning = false;
    private boolean shootSequenceStarted = false;
    private ElapsedTime ballWaitTimer = new ElapsedTime();
    private boolean ballWaitStarted = false;
    private int ballsNeeded = 3;
    private boolean scanningInProgress = false;

    // Shooter settings (fixed for autonomous shoot pose)
    // Shoot pose (49, 84) to blue basket - approximately 30-35 inches
    private double shootRpm = 3250;     // From ShooterLookup for ~30-35"
    private double shootHood = 0.5;     // From ShooterLookup for ~30-35"
    private static final int GOAL_TAG_ID = 20; // blue goal

    // End position shooter settings
    // End pose (55, 209) to blue basket - different angle/distance
    private double endShootRpm = 3400;     // RPM for end position shot
    private double endShootHood = 0.52;    // Hood angle for end position shot

    // Turret auto-aim
    private static final double TURRET_TARGET_DEG = 94;  // Shooting angle for initial shootPose
    private static final double TURRET_END_SHOOT_DEG = 85;  // Shooting angle for endPose
    private static final double TURRET_MOTIF_SCAN_DEG = 100;  // Angle to scan for motif tag
    private static final double TURRET_P_GAIN = 0.006;
    private static final double TURRET_MAX_POWER = 0.30;
    private static final double TURRET_LIMIT_DEG = 5.0;
    private boolean shootingFromEndPose = false;  // Track which shooting position we're using
    
    // Motif scan timing
    private static final double MOTIF_SCAN_TIME_SEC = 1.8;  // How long to scan for motif during drive
    private ElapsedTime motifScanTimer = new ElapsedTime();
    private boolean scanningForMotif = false;  // True while turret scans at 100° for motif
    private boolean turretAtShootAngle = false;  // True when turret reaches 94° after scan
    
    // Auto time management
    private static final double AUTO_TIME_LIMIT_SEC = 30.0;  // Total autonomous time
    private static final double EMERGENCY_ABORT_TIME_SEC = 25.0;  // Abort and park when this time is reached (5 seconds remaining)
    private boolean emergencyAbortTriggered = false;
    
    // Ball detection camera
    private boolean useCameraDetection = true;  // Enable camera-based ball detection
    private ElapsedTime cameraUpdateTimer = new ElapsedTime();  // Throttle camera updates

    // Spindexer unjam detection
    private double lastSpindexerAngle = 0;
    private ElapsedTime spindexerStuckTimer = new ElapsedTime();
    private boolean isUnjamming = false;
    private ElapsedTime unjamTimer = new ElapsedTime();
    private static final double STUCK_THRESHOLD_SEC = 1.0;  // Time before declaring stuck
    private static final double ANGLE_CHANGE_THRESHOLD = 3.0;  // Minimum angle movement to consider "moving"
    private static final double UNJAM_DURATION_SEC = 0.5;  // How long to reverse intake

    // Motif detection
    private int detectedTagId = -1;
    private String[] motifOrder = null;
    private boolean motifLocked = false;

    public enum PathState {
        DRIVE_STARTPOSE_TO_SHOOTPOSE,
        SHOOT_PRELOAD,
        DRIVE_SHOOTPOSE_TO_INTAKE1,
        DRIVE_INTAKE1_TO_INTAKE1POSE2,
        DRIVE_INTAKE1POSE2_TO_INTAKE1POSE3,
        DRIVE_INTAKE1POSE3_TO_INTAKE1POSE4,
        DRIVE_INTAKE1POSE4_TO_SHOOTPOSE,
        SHOOT_INTAKE1,
        DRIVE_SHOOTPOSE_TO_INTAKE2,
        DRIVE_INTAKE2_TO_INTAKE2POSE1,
        DRIVE_INTAKE2POSE1_TO_INTAKE2POSE2,
        DRIVE_INTAKE2POSE2_TO_INTAKE2POSE3,
        DRIVE_INTAKE2POSE3_TO_SHOOTPOSE,
        SHOOT_INTAKE2,
        DRIVE_SHOOTPOSE_TO_ENDPOSE,
        SHOOT_AT_ENDPOSE
    }

    PathState pathState;

    // Poses (matching Blue_Front with 3-stage intake waypoints)
    private final Pose startPose = new Pose(20, 120, Math.toRadians(144));
    private final Pose shootPose = new Pose(49, 84, Math.toRadians(180));
    private final Pose intake1 = new Pose(37, 76, Math.toRadians(180));       // Intake 1 position
    private final Pose intake1Pose2 = new Pose(35.5, 76, Math.toRadians(180));  // Intake 1 waypoint 2
    private final Pose intake1Pose3 = new Pose(30, 76, Math.toRadians(180));  // Intake 1 waypoint 3
    private final Pose intake1Pose4 = new Pose(28, 76, Math.toRadians(180));  // Intake 1 waypoint 4 (2 inches forward)
    private final Pose intake2 = new Pose(45, 56, Math.toRadians(180));       // Intake 2 position
    private final Pose intake2Pose1 = new Pose(37, 56, Math.toRadians(180));  // Intake 2 waypoint 1
    private final Pose intake2Pose2 = new Pose(35, 56, Math.toRadians(180));  // Intake 2 waypoint 2
    private final Pose intake2Pose3 = new Pose(31, 56, Math.toRadians(180));  // Intake 2 waypoint 3
    private final Pose endPose = new Pose(55, 209, Math.toRadians(180));       // Final shoot position

    private boolean pathStarted = false;

    private PathChain driveStartPoseToShootPose;
    private PathChain driveShootPoseToIntake1;
    private PathChain driveIntake1ToIntake1Pose2;
    private PathChain driveIntake1Pose2ToIntake1Pose3;
    private PathChain driveIntake1Pose3ToIntake1Pose4;
    private PathChain driveIntake1Pose4ToShootPose;
    private PathChain driveShootPoseToIntake2;
    private PathChain driveIntake2ToIntake2Pose1;
    private PathChain driveIntake2Pose1ToIntake2Pose2;
    private PathChain driveIntake2Pose2ToIntake2Pose3;
    private PathChain driveIntake2Pose3ToShootPose;
    private PathChain driveShootPoseToEndPose;

    public void buildPaths() {
        driveStartPoseToShootPose = follower.pathBuilder()
                .addPath(new BezierLine(startPose, shootPose))
                .setLinearHeadingInterpolation(startPose.getHeading(), shootPose.getHeading())
                .build();

        driveShootPoseToIntake1 = follower.pathBuilder()
                .addPath(new BezierLine(shootPose, intake1))
                .setConstantHeadingInterpolation(intake1.getHeading())
                .build();

        driveIntake1ToIntake1Pose2 = follower.pathBuilder()
                .addPath(new BezierLine(intake1, intake1Pose2))
                .setConstantHeadingInterpolation(intake1Pose2.getHeading())
                .build();

        driveIntake1Pose2ToIntake1Pose3 = follower.pathBuilder()
                .addPath(new BezierLine(intake1Pose2, intake1Pose3))
                .setConstantHeadingInterpolation(intake1Pose3.getHeading())
                .build();

        driveIntake1Pose3ToIntake1Pose4 = follower.pathBuilder()
                .addPath(new BezierLine(intake1Pose3, intake1Pose4))
                .setConstantHeadingInterpolation(intake1Pose4.getHeading())
                .build();

        driveIntake1Pose4ToShootPose = follower.pathBuilder()
                .addPath(new BezierLine(intake1Pose4, shootPose))
                .setLinearHeadingInterpolation(intake1Pose4.getHeading(), shootPose.getHeading())
                .build();

        driveShootPoseToIntake2 = follower.pathBuilder()
                .addPath(new BezierLine(shootPose, intake2))
                .setConstantHeadingInterpolation(intake2.getHeading())
                .build();

        driveIntake2ToIntake2Pose1 = follower.pathBuilder()
                .addPath(new BezierLine(intake2, intake2Pose1))
                .setConstantHeadingInterpolation(intake2Pose1.getHeading())
                .build();

        driveIntake2Pose1ToIntake2Pose2 = follower.pathBuilder()
                .addPath(new BezierLine(intake2Pose1, intake2Pose2))
                .setConstantHeadingInterpolation(intake2Pose2.getHeading())
                .build();

        driveIntake2Pose2ToIntake2Pose3 = follower.pathBuilder()
                .addPath(new BezierLine(intake2Pose2, intake2Pose3))
                .setConstantHeadingInterpolation(intake2Pose3.getHeading())
                .build();

        driveIntake2Pose3ToShootPose = follower.pathBuilder()
                .addPath(new BezierLine(intake2Pose3, shootPose))
                .setLinearHeadingInterpolation(intake2Pose3.getHeading(), shootPose.getHeading())
                .build();

        driveShootPoseToEndPose = follower.pathBuilder()
                .addPath(new BezierLine(shootPose, endPose))
                .setConstantHeadingInterpolation(endPose.getHeading())
                .build();
    }

    private void updateTurret() {
        // Use different target angle based on whether we're scanning for motif or shooting from endPose
        double shootTargetDeg = shootingFromEndPose ? TURRET_END_SHOOT_DEG : TURRET_TARGET_DEG;
        double targetDeg = scanningForMotif ? TURRET_MOTIF_SCAN_DEG : shootTargetDeg;
            
        int pos = turret.getCurrentPosition();
        double posDeg = pos / TICKS_PER_DEG;
        double targetTicks = targetDeg * TICKS_PER_DEG;
        double error = targetTicks - pos;
        double power = error * TURRET_P_GAIN;
        power = Math.max(-TURRET_MAX_POWER, Math.min(TURRET_MAX_POWER, power));
        if (posDeg > targetDeg + TURRET_LIMIT_DEG && power > 0) {
            power = 0;
        } else if (posDeg < targetDeg - TURRET_LIMIT_DEG && power < 0) {
            power = 0;
        }
        turret.setPower(power);
        
        // Check if turret is at shoot angle (used to gate shooting start)
        turretAtShootAngle = (Math.abs(posDeg - shootTargetDeg) <= TURRET_LIMIT_DEG);
    }

    // Commented out - not needed since we shoot from fixed position in autonomous
    /*
    private void updateShooterFromLimelight() {
        try {
            LLResult result = limelight.getLatestResult();
            if (result != null && result.isValid() && result.getFiducialResults() != null) {
                LLResultTypes.FiducialResult goalTag = null;
                for (LLResultTypes.FiducialResult f : result.getFiducialResults()) {
                    if ((int) f.getFiducialId() == GOAL_TAG_ID) { goalTag = f; break; }
                }
                if (goalTag != null) {
                    Pose3D tagPose = goalTag.getTargetPoseCameraSpace();
                    if (tagPose != null) {
                        double x = tagPose.getPosition().x;
                        double z = tagPose.getPosition().z;
                        double dist = Math.sqrt(x * x + z * z) * 39.3701;
                        if (smoothedDistance < 0) smoothedDistance = dist;
                        else smoothedDistance += SMOOTHING_ALPHA * (dist - smoothedDistance);
                        ShooterLookup.Result tuned = ShooterLookup.lookup(smoothedDistance);
                        shootRpm = tuned.rpm;
                        shootHood = tuned.hoodPos;
                        spindexer.setShootRpm(shootRpm);
                        Servo_Pipeline.hood.setPosition(shootHood);
                    }
                }
            }
        } catch (Exception e) { }
    }
    */

    private double getTickSpeed(double rpm) { return rpm * 28.0 / 60.0; }
    
    /**
     * Check if spindexer is stuck and needs unjamming.
     * Returns true if currently executing unjam routine.
     */
    private boolean checkAndUnjamSpindexer() {
        if (isUnjamming) {
            // Currently unjamming - run reverse intake
            if (unjamTimer.seconds() < UNJAM_DURATION_SEC) {
                // Reverse at half speed
                intake.setPower(0.25);
                intakeMotor2.setPower(-0.5);
                return true;
            } else {
                // Unjam complete - resume normal intake
                isUnjamming = false;
                spindexerStuckTimer.reset();
                lastSpindexerAngle = Servo_Pipeline.spindexerAxon.getRawAngle();
                return false;
            }
        }

        // Check if spindexer is moving during intake
        if (intakeRunning) {
            double currentAngle = Servo_Pipeline.spindexerAxon.getRawAngle();
            double angleDelta = Math.abs(currentAngle - lastSpindexerAngle);
            
            // Handle 360° wraparound
            if (angleDelta > 180) {
                angleDelta = 360 - angleDelta;
            }

            if (angleDelta > ANGLE_CHANGE_THRESHOLD) {
                // Spindexer is moving - reset stuck timer
                spindexerStuckTimer.reset();
                lastSpindexerAngle = currentAngle;
            } else if (spindexerStuckTimer.seconds() >= STUCK_THRESHOLD_SEC) {
                // Spindexer hasn't moved for too long - start unjam
                isUnjamming = true;
                unjamTimer.reset();
                return true;
            }
        }
        
        return false;
    }

    /**
     * Update spindexer slot colors from camera ball detection.
     * Called during intake to provide visual confirmation of ball colors.
     */
    private void updateSpindexerFromCamera() {
        if (!useCameraDetection) return;
        
        // Poll camera once per call
        BallDetectorPipeline.pollOnce();
        List<BallDetection> balls = BallDetectorPipeline.getDetectedBalls();
        
        // Update spindexer slots based on detected balls
        // Ball positions 1-3 map to spindexer slots 0-2
        for (BallDetection ball : balls) {
            int slot = ball.rampPosition - 1;  // Convert 1-based to 0-based
            if (slot >= 0 && slot <= 2) {
                String color = ball.isGreen ? "GREEN" : "PURPLE";
                // Only update if slot is marked as filled
                if (!spindexer.isSlotEmpty(slot)) {
                    spindexer.setSlotColor(slot, color);
                }
            }
        }
    }

    public void statePathUpdate() {
        updateTurret();
        // Shooter settings are fixed for autonomous - no need for continuous Limelight updates
        
        // Emergency abort: if 5 seconds or less remaining, kill current action and go to park
        if (!emergencyAbortTriggered && opmodeTimer.getElapsedTimeSeconds() >= EMERGENCY_ABORT_TIME_SEC) {
            emergencyAbortTriggered = true;
            // Stop all motors
            intake.setPower(0);
            intakeMotor2.setPower(0);
            flywheel.setVelocity(0);
            // Cancel any shooting in progress
            spindexer.resetShootState();
            // Force transition to park
            pathState = PathState.DRIVE_SHOOTPOSE_TO_ENDPOSE;
            pathStarted = false;
            telemetry.addData("EMERGENCY ABORT", "Time limit - parking now!");
        }

        switch (pathState) {
            case DRIVE_STARTPOSE_TO_SHOOTPOSE:
                if (!pathStarted) {
                    follower.followPath(driveStartPoseToShootPose, true);
                    Servo_Pipeline.spindexerAxon.setTargetRotation(SpindexerController.P1);
                    flywheel.setVelocity(getTickSpeed(shootRpm));
                    
                    // Start scanning preload balls during drive
                    spindexer.startScan("GREEN"); // Alliance doesn't matter, we'll use motif order
                    scanningInProgress = true;
                    
                    // Start motif scanning (turret will aim at 30°)
                    scanningForMotif = true;
                    motifScanTimer.reset();
                    
                    pathStarted = true;
                }
                
                // Scan for motif tag while turret is at 40°
                if (scanningForMotif) {
                    if (!motifLocked) {
                        Limelight_Pipeline.pollOnce();
                        int tagId = Limelight_Pipeline.getObeliskTagId();
                        if (tagId != -1) {
                            detectedTagId = tagId;
                            motifOrder = Motif.getShootOrder(tagId);
                            motifLocked = true;
                            telemetry.addData("MOTIF DETECTED", Motif.getMotifName(detectedTagId));
                        }
                    }
                    
                    // Stop scanning after timeout or detection (give full scan time)
                    if (motifLocked || motifScanTimer.seconds() >= MOTIF_SCAN_TIME_SEC) {
                        scanningForMotif = false;
                        if (!motifLocked) {
                            // Fallback if no tag detected
                            motifOrder = Motif.getShootOrder(-1);
                            telemetry.addData("Motif", "FALLBACK - no tag detected");
                        }
                    }
                }
                
                // Continue scanning balls while driving
                if (scanningInProgress && !spindexer.isScanDone()) {
                    spindexer.updateScan();
                } else if (scanningInProgress && spindexer.isScanDone()) {
                    scanningInProgress = false;
                }
                
                // Transition to shoot only when: robot arrived, ball scan done, AND turret at shoot angle
                if (pathStarted && !follower.isBusy() && !scanningInProgress && turretAtShootAngle) {
                    // Robot arrived AND scan complete AND turret ready → build motif queue before shooting
                    if (motifOrder != null) {
                        int[] queue = MotifQueue.buildMotifQueue(
                            motifOrder,
                            spindexer.getSlotColors(),
                            spindexer.getSlotEmptyStatus()
                        );
                        spindexer.setShootQueue(queue);
                    }
                    pathState = PathState.SHOOT_PRELOAD;
                    pathStarted = false;
                } else if (pathStarted && !follower.isBusy() && (!scanningInProgress || !turretAtShootAngle)) {
                    if (scanningInProgress) {
                        telemetry.addLine("Waiting for ball scan to complete...");
                    } else if (!turretAtShootAngle) {
                        telemetry.addLine("Waiting for turret to reach shoot angle...");
                    }
                }
                
                telemetry.addLine("Driving to shoot pose...");
                if (scanningForMotif) {
                    telemetry.addData("Scanning motif", String.format("%.1fs / %.1fs", 
                        motifScanTimer.seconds(), MOTIF_SCAN_TIME_SEC));
                }
                if (scanningInProgress) {
                    telemetry.addData("Scanning preload", spindexer.getScanSlot() + 1 + "/3");
                } else if (motifLocked) {
                    telemetry.addData("Motif ready", Motif.getMotifName(detectedTagId));
                }
                if (!turretAtShootAngle && !scanningForMotif) {
                    telemetry.addData("Turret", "Moving to shoot angle");
                }
                break;

            case SHOOT_PRELOAD:
                if (!shootSequenceStarted) {
                    spindexer.updateShoot(true, false, flywheel);
                    intake.setPower(-0.5);
                    intakeMotor2.setPower(1.0);
                    shootSequenceStarted = true;
                } else {
                    spindexer.updateShoot(false, false, flywheel);
                }

                if (!spindexer.isShooting() && shootSequenceStarted) {
                    intake.setPower(0);
                    intakeMotor2.setPower(0);
                    flywheel.setVelocity(0);
                    shootSequenceStarted = false;
                    int confirmed = spindexer.getConfirmedShots();
                    spindexer.resetShootState();
                    spindexer.clearAllSlots();
                    spindexer.goToSlot(0);  // Reset to intake position P1
                    Servo_Pipeline.flicker.setPosition(0);

                    ballsNeeded = (confirmed <= 0) ? 0 : 3;
                    pathState = PathState.DRIVE_SHOOTPOSE_TO_INTAKE1;
                    pathStarted = false;
                }
                telemetry.addLine("Preload Shot — Motif: " + (motifLocked ? Motif.getMotifName(detectedTagId) : "FALLBACK"));
                telemetry.addData("Confirmed Shots", spindexer.getConfirmedShots());
                break;

            case DRIVE_SHOOTPOSE_TO_INTAKE1:
                if (!pathStarted) {
                    follower.followPath(driveShootPoseToIntake1, 0.625, true);
                    pathStarted = true;

                    intake.setPower(-0.5);
                    intakeMotor2.setPower(1.0);
                    intakeRunning = true;
                    
                    // Initialize unjam tracking
                    spindexerStuckTimer.reset();
                    lastSpindexerAngle = Servo_Pipeline.spindexerAxon.getRawAngle();
                }

                if (pathStarted && !follower.isBusy()) {
                    if (!ballWaitStarted) {
                        ballWaitTimer.reset();
                        ballWaitStarted = true;
                    }
                    if (Sensor.isBallPresent() || ballWaitTimer.seconds() >= 1.0) {
                        ballsNeeded--;
                        if (ballsNeeded <= 0) {
                            // Got all we need — skip remaining intake waypoints
                            pathState = PathState.DRIVE_INTAKE1POSE4_TO_SHOOTPOSE;
                        } else {
                            pathState = PathState.DRIVE_INTAKE1_TO_INTAKE1POSE2;
                        }
                        pathStarted = false;
                        ballWaitStarted = false;
                    }
                }
                break;

            case DRIVE_INTAKE1_TO_INTAKE1POSE2:
                if (!pathStarted) {
                    follower.followPath(driveIntake1ToIntake1Pose2, 0.625, true);
                    pathStarted = true;

                    intake.setPower(-0.5);
                    intakeMotor2.setPower(1.0);
                    intakeRunning = true;
                    
                    // Initialize unjam tracking
                    spindexerStuckTimer.reset();
                    lastSpindexerAngle = Servo_Pipeline.spindexerAxon.getRawAngle();
                }

                if (pathStarted && !follower.isBusy()) {
                    if (!ballWaitStarted) {
                        ballWaitTimer.reset();
                        ballWaitStarted = true;
                    }
                    if (Sensor.isBallPresent() || ballWaitTimer.seconds() >= 1.0) {
                        ballsNeeded--;
                        if (ballsNeeded <= 0) {
                            pathState = PathState.DRIVE_INTAKE1POSE4_TO_SHOOTPOSE;
                        } else {
                            pathState = PathState.DRIVE_INTAKE1POSE2_TO_INTAKE1POSE3;
                        }
                        pathStarted = false;
                        ballWaitStarted = false;
                    }
                }
                break;

            case DRIVE_INTAKE1POSE2_TO_INTAKE1POSE3:
                if (!pathStarted) {
                    follower.followPath(driveIntake1Pose2ToIntake1Pose3, 0.625, true);
                    pathStarted = true;

                    intake.setPower(-0.5);
                    intakeMotor2.setPower(1.0);
                    intakeRunning = true;
                    
                    // Initialize unjam tracking
                    spindexerStuckTimer.reset();
                    lastSpindexerAngle = Servo_Pipeline.spindexerAxon.getRawAngle();
                }

                if (pathStarted && !follower.isBusy()) {
                    if (!ballWaitStarted) {
                        ballWaitTimer.reset();
                        ballWaitStarted = true;
                    }
                    if (Sensor.isBallPresent() || ballWaitTimer.seconds() >= 1.0) {
                        ballsNeeded--;
                        pathState = PathState.DRIVE_INTAKE1POSE3_TO_INTAKE1POSE4;
                        pathStarted = false;
                        ballWaitStarted = false;
                    }
                }
                break;

            case DRIVE_INTAKE1POSE3_TO_INTAKE1POSE4:
                if (!pathStarted) {
                    follower.followPath(driveIntake1Pose3ToIntake1Pose4, 0.625, true);
                    pathStarted = true;

                    intake.setPower(-0.5);
                    intakeMotor2.setPower(1.0);
                    intakeRunning = true;
                    
                    // Initialize unjam tracking
                    spindexerStuckTimer.reset();
                    lastSpindexerAngle = Servo_Pipeline.spindexerAxon.getRawAngle();
                }

                if (pathStarted && !follower.isBusy()) {
                    if (!ballWaitStarted) {
                        ballWaitTimer.reset();
                        ballWaitStarted = true;
                    }
                    if (Sensor.isBallPresent() || ballWaitTimer.seconds() >= 1.0) {
                        ballsNeeded--;
                        pathState = PathState.DRIVE_INTAKE1POSE4_TO_SHOOTPOSE;
                        pathStarted = false;
                        ballWaitStarted = false;
                    }
                }
                break;

            case DRIVE_INTAKE1POSE4_TO_SHOOTPOSE:
                if (!pathStarted) {
                    follower.followPath(driveIntake1Pose4ToShootPose, true);
                    flywheel.setVelocity(getTickSpeed(shootRpm));
                    
                    // Start color scan during drive to identify balls
                    spindexer.startScan("GREEN");
                    scanningInProgress = true;
                    
                    pathStarted = true;
                }
                
                // Continue scanning while driving
                if (scanningInProgress && !spindexer.isScanDone()) {
                    spindexer.updateScan();
                } else if (scanningInProgress && spindexer.isScanDone()) {
                    scanningInProgress = false;
                }
                
                if (follower.getCurrentTValue() >= 0.5) {
                    intake.setPower(0);
                    intakeMotor2.setPower(0);
                    intakeRunning = false;
                }
                
                if (pathStarted && !follower.isBusy() && !scanningInProgress) {
                    // Robot arrived AND scan complete → build motif queue before shooting
                    if (motifOrder != null) {
                        int[] queue = MotifQueue.buildMotifQueue(
                            motifOrder,
                            spindexer.getSlotColors(),
                            spindexer.getSlotEmptyStatus()
                        );
                        spindexer.setShootQueue(queue);
                    }
                    pathState = PathState.SHOOT_INTAKE1;
                    shootingFromEndPose = false;  // Reset for initial shoot position
                    pathStarted = false;
                }
                
                telemetry.addLine("Returning to shoot pose...");
                if (scanningInProgress) {
                    telemetry.addData("Scanning slot", spindexer.getScanSlot() + 1);
                } else if (!follower.isBusy()) {
                    telemetry.addData("Scan", "Complete - Ready!");
                } else {
                    telemetry.addData("Scan", "Complete - Driving...");
                }
                break;

            case SHOOT_INTAKE1:
                if (!shootSequenceStarted) {
                    spindexer.updateShoot(true, false, flywheel);
                    intake.setPower(-0.5);
                    intakeMotor2.setPower(1.0);
                    shootSequenceStarted = true;
                } else {
                    spindexer.updateShoot(false, false, flywheel);
                }

                if (!spindexer.isShooting() && shootSequenceStarted) {
                    intake.setPower(0);
                    intakeMotor2.setPower(0);
                    flywheel.setVelocity(0);
                    shootSequenceStarted = false;
                    ballsNeeded = spindexer.getConfirmedShots();
                    spindexer.resetShootState();
                    spindexer.clearAllSlots();
                    spindexer.goToSlot(0);  // Reset to intake position P1
                    Servo_Pipeline.flicker.setPosition(0);

                    pathState = (ballsNeeded <= 0)
                            ? PathState.DRIVE_SHOOTPOSE_TO_ENDPOSE
                            : PathState.DRIVE_SHOOTPOSE_TO_INTAKE2;
                }
                telemetry.addLine("Sample 1 Shot — Motif: " + (motifLocked ? Motif.getMotifName(detectedTagId) : "FALLBACK"));
                telemetry.addData("Confirmed Shots", spindexer.getConfirmedShots());
                break;

            case DRIVE_SHOOTPOSE_TO_INTAKE2:
                if (!pathStarted) {
                    follower.followPath(driveShootPoseToIntake2, true);
                    pathStarted = true;
                }
                if (pathStarted && !follower.isBusy()) {
                    pathState = PathState.DRIVE_INTAKE2_TO_INTAKE2POSE1;
                    pathStarted = false;
                }
                break;

            case DRIVE_INTAKE2_TO_INTAKE2POSE1:
                if (!pathStarted) {
                    follower.followPath(driveIntake2ToIntake2Pose1, 0.625, true);
                    pathStarted = true;

                    intake.setPower(-0.5);
                    intakeMotor2.setPower(1.0);
                    intakeRunning = true;
                    
                    // Initialize unjam tracking
                    spindexerStuckTimer.reset();
                    lastSpindexerAngle = Servo_Pipeline.spindexerAxon.getRawAngle();
                }

                if (pathStarted && !follower.isBusy()) {
                    if (!ballWaitStarted) {
                        ballWaitTimer.reset();
                        ballWaitStarted = true;
                    }
                    if (Sensor.isBallPresent() || ballWaitTimer.seconds() >= 1.0) {
                        ballsNeeded--;
                        if (ballsNeeded <= 0) {
                            pathState = PathState.DRIVE_INTAKE2POSE3_TO_SHOOTPOSE;
                        } else {
                            pathState = PathState.DRIVE_INTAKE2POSE1_TO_INTAKE2POSE2;
                        }
                        pathStarted = false;
                        ballWaitStarted = false;
                    }
                }
                break;

            case DRIVE_INTAKE2POSE1_TO_INTAKE2POSE2:
                if (!pathStarted) {
                    follower.followPath(driveIntake2Pose1ToIntake2Pose2, 0.625, true);
                    pathStarted = true;

                    intake.setPower(-0.5);
                    intakeMotor2.setPower(1.0);
                    intakeRunning = true;
                    
                    // Initialize unjam tracking
                    spindexerStuckTimer.reset();
                    lastSpindexerAngle = Servo_Pipeline.spindexerAxon.getRawAngle();
                }

                if (pathStarted && !follower.isBusy()) {
                    if (!ballWaitStarted) {
                        ballWaitTimer.reset();
                        ballWaitStarted = true;
                    }
                    if (Sensor.isBallPresent() || ballWaitTimer.seconds() >= 1.0) {
                        ballsNeeded--;
                        if (ballsNeeded <= 0) {
                            pathState = PathState.DRIVE_INTAKE2POSE3_TO_SHOOTPOSE;
                        } else {
                            pathState = PathState.DRIVE_INTAKE2POSE2_TO_INTAKE2POSE3;
                        }
                        pathStarted = false;
                        ballWaitStarted = false;
                    }
                }
                break;

            case DRIVE_INTAKE2POSE2_TO_INTAKE2POSE3:
                if (!pathStarted) {
                    follower.followPath(driveIntake2Pose2ToIntake2Pose3, 0.625, true);
                    pathStarted = true;

                    intake.setPower(-0.5);
                    intakeMotor2.setPower(1.0);
                    intakeRunning = true;
                    
                    // Initialize unjam tracking
                    spindexerStuckTimer.reset();
                    lastSpindexerAngle = Servo_Pipeline.spindexerAxon.getRawAngle();
                }

                if (pathStarted && !follower.isBusy()) {
                    if (!ballWaitStarted) {
                        ballWaitTimer.reset();
                        ballWaitStarted = true;
                    }
                    if (Sensor.isBallPresent() || ballWaitTimer.seconds() >= 1.0) {
                        ballsNeeded--;
                        pathState = PathState.DRIVE_INTAKE2POSE3_TO_SHOOTPOSE;
                        pathStarted = false;
                        ballWaitStarted = false;
                    }
                }
                break;

            case DRIVE_INTAKE2POSE3_TO_SHOOTPOSE:
                if (!pathStarted) {
                    follower.followPath(driveIntake2Pose3ToShootPose, true);
                    flywheel.setVelocity(getTickSpeed(shootRpm));
                    
                    // Start color scan during drive to identify balls
                    spindexer.startScan("GREEN");
                    scanningInProgress = true;
                    
                    pathStarted = true;
                }
                
                // Continue scanning while driving
                if (scanningInProgress && !spindexer.isScanDone()) {
                    spindexer.updateScan();
                } else if (scanningInProgress && spindexer.isScanDone()) {
                    scanningInProgress = false;
                }
                
                if (follower.getCurrentTValue() >= 0.5) {
                    intake.setPower(0);
                    intakeMotor2.setPower(0);
                    intakeRunning = false;
                }
                if (pathStarted && !follower.isBusy() && !scanningInProgress) {
                    // Robot arrived AND scan complete → build motif queue before shooting
                    if (motifOrder != null) {
                        int[] queue = MotifQueue.buildMotifQueue(
                            motifOrder,
                            spindexer.getSlotColors(),
                            spindexer.getSlotEmptyStatus()
                        );
                        spindexer.setShootQueue(queue);
                    }
                    pathState = PathState.SHOOT_INTAKE2;
                    shootingFromEndPose = false;  // Reset for initial shoot position
                    pathStarted = false;
                }
                
                telemetry.addLine("Returning to shoot pose...");
                if (scanningInProgress) {
                    telemetry.addData("Scanning slot", spindexer.getScanSlot() + 1);
                } else if (!follower.isBusy()) {
                    telemetry.addData("Scan", "Complete - Ready!");
                } else {
                    telemetry.addData("Scan", "Complete - Driving...");
                }
                break;

            case SHOOT_INTAKE2:
                if (!shootSequenceStarted) {
                    spindexer.updateShoot(true, false, flywheel);
                    intake.setPower(-0.5);
                    intakeMotor2.setPower(1.0);
                    shootSequenceStarted = true;
                } else {
                    spindexer.updateShoot(false, false, flywheel);
                }

                if (!spindexer.isShooting() && shootSequenceStarted) {
                    intake.setPower(0);
                    intakeMotor2.setPower(0);
                    flywheel.setVelocity(0);
                    shootSequenceStarted = false;
                    spindexer.resetShootState();
                    spindexer.clearAllSlots();
                    spindexer.goToSlot(0);  // Reset to intake position P1 (for end state)
                    Servo_Pipeline.flicker.setPosition(0);
                    pathState = PathState.DRIVE_SHOOTPOSE_TO_ENDPOSE;
                }
                telemetry.addLine("Sample 2 Shot — Motif: " + (motifLocked ? Motif.getMotifName(detectedTagId) : "FALLBACK"));
                telemetry.addData("Confirmed Shots", spindexer.getConfirmedShots());
                break;

            case DRIVE_SHOOTPOSE_TO_ENDPOSE:
                if (!pathStarted) {
                    follower.followPath(driveShootPoseToEndPose, true);
                    shootingFromEndPose = true;  // Enable end position shooter settings
                    flywheel.setVelocity(getTickSpeed(endShootRpm));
                    Servo_Pipeline.hood.setPosition(endShootHood);
                    pathStarted = true;
                }
                if (pathStarted && !follower.isBusy() && turretAtShootAngle) {
                    pathState = PathState.SHOOT_AT_ENDPOSE;
                    pathStarted = false;
                } else if (pathStarted && !follower.isBusy() && !turretAtShootAngle) {
                    telemetry.addLine("Waiting for turret to reach end shoot angle...");
                }
                telemetry.addLine("Driving to end position...");
                telemetry.addData("End Shoot Angle", String.format("%.1f°", TURRET_END_SHOOT_DEG));
                break;

            case SHOOT_AT_ENDPOSE:
                if (!shootSequenceStarted) {
                    spindexer.updateShoot(true, false, flywheel);
                    intake.setPower(-0.5);
                    intakeMotor2.setPower(1.0);
                    shootSequenceStarted = true;
                } else {
                    spindexer.updateShoot(false, false, flywheel);
                }

                if (!spindexer.isShooting() && shootSequenceStarted) {
                    intake.setPower(0);
                    intakeMotor2.setPower(0);
                    flywheel.setVelocity(0);
                    shootSequenceStarted = false;
                    int confirmed = spindexer.getConfirmedShots();
                    spindexer.resetShootState();
                    telemetry.addLine("End position shots complete — Auto finished");
                    turret.setPower(0);
                    requestOpModeStop();
                }
                telemetry.addLine("Shooting from end position");
                telemetry.addData("Shot", spindexer.getConfirmedShots() + "/3");
                break;

            default:
                telemetry.addLine("No valid path state");
                break;
        }
    }

    @Override
    public void init() {
        pathState = PathState.DRIVE_STARTPOSE_TO_SHOOTPOSE;  // Start by driving
        pathTimer = new Timer();
        opmodeTimer = new Timer();
        opmodeTimer.resetTimer();
        follower = Constants.createFollower(hardwareMap);

        motorPipeline = new Motor_PipeLine(this);
        Motor_PipeLine.resetMotors();
        flywheel      = Motor_PipeLine.flywheel;
        intake        = Motor_PipeLine.intake;
        intakeMotor2  = Motor_PipeLine.intake2;
        turret        = Motor_PipeLine.turret;

        Sensor.initSensors(this);
        servoPipeline = new Servo_Pipeline(this);
        spindexer = new SpindexerController();
        spindexer.setFlickerPositions(0, 0.375);
        spindexer.setShootRpm(shootRpm);
        spindexer.prefillAllSlots();
        
        // Initialize ball detection camera
        try {
            BallDetectorPipeline.init(this, "logitech");
            telemetry.addData("Ball Camera", "Initialized");
        } catch (Exception e) {
            useCameraDetection = false;
            telemetry.addData("Ball Camera", "Failed - using color sensors only");
        }
        
        // Initialize ball detection camera
        try {
            BallDetectorPipeline.init(this, "logitech");
            telemetry.addData("Ball Camera", "Initialized");
        } catch (Exception e) {
            useCameraDetection = false;
            telemetry.addData("Ball Camera", "Failed - using color sensors only");
        }

        // Initialize Limelight for obelisk motif detection during init
        Limelight_Pipeline.initLimelight(this);

        buildPaths();
        follower.setPose(startPose);

        telemetry.addData("Status", "Initialized — scanning for obelisk...");
        telemetry.update();
    }

    /**
     * Runs repeatedly after init() until play is pressed.
     * Scans for the obelisk tag and locks the motif.
     */
    @Override
    public void init_loop() {
        Limelight_Pipeline.pollOnce();
        int tagId = Limelight_Pipeline.getObeliskTagId();

        if (tagId != -1) {
            detectedTagId = tagId;
            motifOrder = Motif.getShootOrder(tagId);
            motifLocked = true;
        }

        // Show detection status
        if (motifLocked) {
            telemetry.addData("MOTIF LOCKED", Motif.getMotifName(detectedTagId));
            telemetry.addData("Shoot Order",
                    motifOrder[0] + " → " + motifOrder[1] + " → " + motifOrder[2]);
        } else {
            telemetry.addData("Scanning...", "No obelisk tag found yet");
            telemetry.addData("Looking for", "Tag 21, 22, or 23");
        }
        telemetry.update();
    }

    @Override
    public void start() {
        Servo_Pipeline.flicker.setPosition(0);
        Servo_Pipeline.spindexerAxon.setTargetRotation(SpindexerController.P1);
        Servo_Pipeline.hood.setPosition(shootHood);  // Use fixed hood position for autonomous
        opmodeTimer.resetTimer();
        pathState = PathState.DRIVE_STARTPOSE_TO_SHOOTPOSE;  // Start by driving (motif scan happens during drive)
        spindexer.goToSlot(0);

        // Apply motif to spindexer if already locked during init_loop
        if (motifLocked && motifOrder != null) {
            spindexer.setMotifOrder(motifOrder);
        }
        // Note: fallback is now handled during DRIVE_STARTPOSE_TO_SHOOTPOSE if motif not detected
    }

    @Override
    public void loop() {
        follower.update();
        Sensor.updateSensors();  // Update color sensors
        
        // Check for spindexer jam and run unjam routine if needed
        boolean unjamming = checkAndUnjamSpindexer();
        
        // Update spindexer from camera periodically (every 200ms to avoid lag)
        if (useCameraDetection && cameraUpdateTimer.milliseconds() >= 200) {
            updateSpindexerFromCamera();
            cameraUpdateTimer.reset();
        }
        
        statePathUpdate();
        
        // Only update intake state machine if not unjamming
        if (!unjamming) {
            spindexer.updateIntake(intakeRunning);
        }

        telemetry.addData("Motif", motifLocked ? Motif.getMotifName(detectedTagId) : "FALLBACK (shoot any order)");
        telemetry.addData("Path state", pathState.toString());
        
        if (unjamming) {
            telemetry.addData("*** UNJAMMING ***", String.format("%.1fs", unjamTimer.seconds()));
        }
        
        // Show camera ball detections
        if (useCameraDetection) {
            telemetry.addData("Camera Balls", BallDetectorPipeline.getBallSequence());
        }
        
        telemetry.addData("X", follower.getPose().getX());
        telemetry.addData("Y", follower.getPose().getY());
        telemetry.addData("Heading", Math.toDegrees(follower.getPose().getHeading()));
        telemetry.addData("Path time (s): ", pathTimer.getElapsedTimeSeconds());
        telemetry.update();
    }
}
