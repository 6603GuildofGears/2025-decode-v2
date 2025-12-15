package org.firstinspires.ftc.teamcode.pedroPathing;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.*;
import com.pedropathing.paths.*;
import com.pedropathing.util.*;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

// Limelight SDK imports
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import java.util.List;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

@Disabled
@Autonomous(name = "Blue - Close (Goal)", group = "Competition")
public class RedCloseAuto extends OpMode {

    private Follower follower;
    private Timer pathTimer, opmodeTimer, actionTimer;
    private int pathState;
    private int scoreState;

    // Hardware - Use DcMotorEx for all motors for consistency
    private DcMotorEx shooter;  // Note: in hardware config this is labeled "intakeMotor"
    private DcMotorEx intake;   // Note: in hardware config this is labeled "shooterMotor"
    private Servo blocker;
    
    // Limelight for position correction
    private Limelight3A limelight;
    
    // Sensor fusion variables
    private double lastOdoX = 0;
    private double lastOdoY = 0;
    public static double LIMELIGHT_WEIGHT = 0.85;
    private boolean limelightActive = false;
    private int limelightUpdateCount = 0;
    private int tagIdSeen = -1;
    private double tagDistance = 0;

    private final double TICKS_PER_REV = 28;
    
    // Safety timeouts
    private static final double MAX_PATH_TIME = 4.0;  // Max seconds per path
    private static final double MAX_AUTO_TIME = 29.5;  // Stop before 30s auto period ends
    private static final double MAX_LIMELIGHT_CORRECTION = 12.0;  // Max inches to correct in one update
    private static final double FIELD_MIN = 0.0;
    private static final double FIELD_MAX = 141.24;
    
    // Scoring constants
    public static double SHOOTER_RPM = 3700;  // Shooter speed from Rango
    public static double INTAKE_PICKUP_POWER = 0.875;  // Power to pick up from ground
    public static double INTAKE_FEED_POWER = 0.625;  // Power to feed into shooter
    public static double RPM_TOLERANCE = 100;  // Shooter is ready within this RPM of target
    private static final double REV_UP_TIME = 3.0;  // Seconds to rev up shooter (for initial preload)
    private static final double PUSH_TIME = 2.5;  // Seconds per push pulse
    private static final double PAUSE_TIME = 0.3;  // Pause between pulses
    private static final double SETTLE_TIME = 0.5;  // Stop to re-aim before shooting
    private static final int SAMPLES_PER_SCORE = 2;  // Score 2 samples each time
    private static final int PUSH_PULSES = 3;  // Number of push pulses
    private int currentPulse = 0;  // Current pulse counter
    
    // Servo positions
    private static final double BLOCKER_OPEN = 0.175;  // From Rango
    private static final double BLOCKER_CLOSED = 0.3;  // From Rango
    
    // AprilTag positions for localization (Red alliance)
    private static final double[][] RED_TAG_POSITIONS = {
        {135.24, 47.25},   // Tag 24 - Red Goal
        {135.24, 70.62},   // Tag 25 - Motif 21
        {135.24, 93.99},   // Tag 26 - Motif 22
        {135.24, 117.36}   // Tag 27 - Motif 23
    };

    // Poses for Red Close starting position (Observation Zone - audience wall)
    public final Pose startPose = new Pose(81.5, 131.5, Math.toRadians(0));  // Audience wall, facing toward field
    public final Pose shootPose = new Pose(75, 81, Math.toRadians(45));  // Shooting position near red processor
    public final Pose intakePose1 = new Pose(130, 84, Math.toRadians(180));  // First sample - butt facing sample
    public final Pose intakePose2Bridge = new Pose(85, 60, Math.toRadians(135));  // Bridge for sample 2
    public final Pose intakePose2 = new Pose(130, 59, Math.toRadians(180));  // Second sample - butt facing sample
    public final Pose intakePose3 = new Pose(85, 37, Math.toRadians(90));  // Third sample intermediate
    public final Pose intakePose3b = new Pose(130, 35, Math.toRadians(180));  // Third sample - butt facing sample

    // Paths
    private Path scorePreload;
    private PathChain grabSample1, scoreSample1, grabSample2, grabSample2b, scoreSample2, grabSample3, grabSample3b, scoreSample3;

    /**
     * Build all autonomous paths
     */
    public void buildPaths() {
        // Score preloaded sample in high basket
        scorePreload = new Path(new BezierLine(startPose, shootPose));
        scorePreload.setLinearHeadingInterpolation(startPose.getHeading(), shootPose.getHeading());

        // Grab first sample from spike marks
        grabSample1 = follower.pathBuilder()
                .addPath(new BezierLine(shootPose, intakePose1))
                .setTangentHeadingInterpolation()
                .build();

        // Return to score first sample
        scoreSample1 = follower.pathBuilder()
                .addPath(new BezierLine(intakePose1, shootPose))
                .setTangentHeadingInterpolation()
                .build();

        // Grab second sample - L-shaped path with bridge
        grabSample2 = follower.pathBuilder()
                .addPath(new BezierLine(shootPose, intakePose2Bridge))
                .setTangentHeadingInterpolation()
                .build();

        // Complete the L to sample 2
        grabSample2b = follower.pathBuilder()
                .addPath(new BezierLine(intakePose2Bridge, intakePose2))
                .setTangentHeadingInterpolation()
                .build();

        // Score second sample
        scoreSample2 = follower.pathBuilder()
                .addPath(new BezierLine(intakePose2, shootPose))
                .setTangentHeadingInterpolation()
                .build();

        // Grab third sample - goes to intermediate position first
        grabSample3 = follower.pathBuilder()
                .addPath(new BezierLine(shootPose, intakePose3))
                .setTangentHeadingInterpolation()
                .build();
        
        // Move to actual sample position
        grabSample3b = follower.pathBuilder()
                .addPath(new BezierLine(intakePose3, intakePose3b))
                .setTangentHeadingInterpolation()
                .build();

        // Score third sample
        scoreSample3 = follower.pathBuilder()
                .addPath(new BezierLine(intakePose3b, shootPose))
                .setTangentHeadingInterpolation()
                .build();
    }

    /**
     * Score samples into basket
     * Settles for 0.5s, revs shooter for 3s, opens blocker, runs intake for 3s
     * Repeats for 2 samples total
     */
    private void scoreSequence() {
        switch (scoreState) {
            case 0: // Settle/re-aim before shooting (0.5 seconds)
                blocker.setPosition(BLOCKER_CLOSED);
                shooter.setVelocity(0);
                intake.setPower(0);
                actionTimer.resetTimer();
                scoreState++;
                break;
                
            case 1: // Wait to settle
                if (actionTimer.getElapsedTimeSeconds() >= SETTLE_TIME) {
                    shooter.setVelocity(getTickSpeed(SHOOTER_RPM));
                    actionTimer.resetTimer();
                    scoreState++;
                }
                break;
                
            case 2: // Wait for shooter to rev up (3 seconds)
                if (actionTimer.getElapsedTimeSeconds() >= REV_UP_TIME) {
                    blocker.setPosition(BLOCKER_OPEN);
                    intake.setPower(INTAKE_FEED_POWER);  // Slower power to feed into shooter
                    actionTimer.resetTimer();
                    scoreState++;
                }
                break;
                
            case 3: // Push first sample through (3 seconds)
                if (actionTimer.getElapsedTimeSeconds() >= PUSH_TIME) {
                    intake.setPower(0);
                    blocker.setPosition(BLOCKER_CLOSED);
                    actionTimer.resetTimer();
                    scoreState++;
                }
                break;
                
            case 4: // Rev up for second sample (3 seconds)
                if (actionTimer.getElapsedTimeSeconds() >= REV_UP_TIME) {
                    blocker.setPosition(BLOCKER_OPEN);
                    intake.setPower(INTAKE_FEED_POWER);  // Slower power to feed into shooter
                    actionTimer.resetTimer();
                    scoreState++;
                }
                break;
                
            case 5: // Push second sample through (3 seconds)
                if (actionTimer.getElapsedTimeSeconds() >= PUSH_TIME) {
                    intake.setPower(0);
                    blocker.setPosition(BLOCKER_CLOSED);
                    shooter.setVelocity(0);
                    scoreState++;
                }
                break;
                
            case 6: // Scoring complete
                // Wait for next state to reset
                break;
        }
    }
    
    /**
     * Check if scoring is complete
     */
    private boolean scoringComplete() {
        return scoreState >= 5;
    }
    
    /**
     * Reset scoring sequence for next use
     */
    private void resetScoring() {
        scoreState = 0;
    }

    /**
     * Main autonomous state machine
     */
    public void autonomousPathUpdate() {
        switch (pathState) {
            case 0: // Start - drive to basket (shooter already revving)
                follower.followPath(scorePreload);
                setPathState(1);
                break;

            case 1: // Score preload (2 samples) - skip settle & rev
                if (!follower.isBusy()) {
                    if (scoreState == 0) {
                        scoreState = 2;  // Skip settle and rev - already revving from start()
                    }
                    scoreSequence();
                    if (scoringComplete()) {
                        resetScoring();
                        intake.setPower(INTAKE_PICKUP_POWER);  // Full power for ground pickup
                        follower.followPath(grabSample1, true);
                        setPathState(2);
                    }
                }
                break;

            case 2: // Driving to first sample (intake running)
                if (!follower.isBusy()) {
                    intake.setPower(0);
                    follower.followPath(scoreSample1, true);
                    setPathState(3);
                }
                break;

            case 3: // Return to shoot position, start revving when close
                // Check if we're close to target (within ~10 inches) and start revving
                if (!follower.isBusy()) {
                    // Path complete, check if shooter is at speed
                    shooter.setVelocity(getTickSpeed(SHOOTER_RPM));
                    if (isShooterAtSpeed()) {
                        setPathState(4);
                    }
                } else {
                    double distToTarget = Math.hypot(
                        follower.getPose().getX() - shootPose.getX(),
                        follower.getPose().getY() - shootPose.getY()
                    );
                    if (distToTarget < 10) {
                        shooter.setVelocity(getTickSpeed(SHOOTER_RPM));
                    }
                }
                break;

            case 4: // Score first sample (shooter already at speed)
                if (scoreState == 0) {
                    scoreState = 2;  // Skip settle and rev - already done
                }
                scoreSequence();
                if (scoringComplete()) {
                    resetScoring();
                    intake.setPower(INTAKE_PICKUP_POWER);  // Full power for ground pickup
                    follower.followPath(grabSample2, true);
                    setPathState(5);
                }
                break;

            case 5: // Driving to sample 2 bridge (intake running)
                if (!follower.isBusy()) {
                    follower.followPath(grabSample2b, true);
                    setPathState(6);
                }
                break;

            case 6: // Completing sample 2 path (intake still running)
                if (!follower.isBusy()) {
                    intake.setPower(0);
                    follower.followPath(scoreSample2, true);
                    setPathState(7);
                }
                break;

            case 7: // Return to shoot position, start revving when close
                if (!follower.isBusy()) {
                    shooter.setVelocity(getTickSpeed(SHOOTER_RPM));
                    if (isShooterAtSpeed()) {
                        setPathState(8);
                    }
                } else {
                    double distToTarget = Math.hypot(
                        follower.getPose().getX() - shootPose.getX(),
                        follower.getPose().getY() - shootPose.getY()
                    );
                    if (distToTarget < 10) {
                        shooter.setVelocity(getTickSpeed(SHOOTER_RPM));
                    }
                }
                break;

            case 8: // Score second sample (shooter already at speed)
                if (scoreState == 0) {
                    scoreState = 2;  // Skip settle and rev - already done
                }
                scoreSequence();
                if (scoringComplete()) {
                    resetScoring();
                    intake.setPower(INTAKE_PICKUP_POWER);  // Full power for ground pickup
                    follower.followPath(grabSample3, true);
                    setPathState(9);
                }
                break;

            case 9: // Move to intermediate position for third sample (intake running)
                if (!follower.isBusy()) {
                    // Keep intake running through both paths
                    follower.followPath(grabSample3b, true);
                    setPathState(10);
                }
                break;

            case 10: // Driving to third sample (intake still running)
                if (!follower.isBusy()) {
                    intake.setPower(0);
                    follower.followPath(scoreSample3, true);
                    setPathState(11);
                }
                break;

            case 11: // Return to shoot position, start revving when close
                if (!follower.isBusy()) {
                    shooter.setVelocity(getTickSpeed(SHOOTER_RPM));
                    if (isShooterAtSpeed()) {
                        setPathState(12);
                    }
                } else {
                    double distToTarget = Math.hypot(
                        follower.getPose().getX() - shootPose.getX(),
                        follower.getPose().getY() - shootPose.getY()
                    );
                    if (distToTarget < 10) {
                        shooter.setVelocity(getTickSpeed(SHOOTER_RPM));
                    }
                }
                break;

            case 12: // Score third sample (shooter already revved)
                if (scoreState == 0) {
                    scoreState = 2;  // Skip settle and rev - already done
                }
                scoreSequence();
                if (scoringComplete()) {
                    setPathState(-1); // Done - no parking
                }
                break;
        }
    }

    /**
     * Change path state and reset timer
     */
    public void setPathState(int pState) {
        pathState = pState;
        pathTimer.resetTimer();
    }

    @Override
    public void init() {
        pathTimer = new Timer();
        opmodeTimer = new Timer();
        actionTimer = new Timer();

        // Initialize follower with Pedro Pathing
        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(startPose);


        // Hardware mapping - NOTE: shooter/intake are swapped in hardware config
        shooter = hardwareMap.get(DcMotorEx.class, "intakeMotor");  // Actual shooter motor
        intake = hardwareMap.get(DcMotorEx.class, "shooterMotor");  // Actual intake motor
        blocker = hardwareMap.get(Servo.class, "blocker");

        // Shooter setup (this is the actual shooter, mapped to "intakeMotor")
        shooter.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        shooter.setDirection(DcMotorSimple.Direction.REVERSE);

        // Intake setup (this is the actual intake, mapped to "shooterMotor")
        intake.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        intake.setDirection(DcMotorSimple.Direction.REVERSE);
        
        // Limelight setup - removed for simple test
        
        // Build paths
        buildPaths();
        
        // Force an update to get initial pose from Pinpoint
        follower.update();

        telemetry.addLine("Blue - Close (SIMPLE TEST)");
        telemetry.addLine("NOTE: Shooter is 'intakeMotor', Intake is 'shooterMotor'");
        telemetry.addLine("---");
        telemetry.addData("Start Pose Set To", "X=%.1f Y=%.1f H=%.0f°", startPose.getX(), startPose.getY(), Math.toDegrees(startPose.getHeading()));
        telemetry.addData("Pinpoint Reading", "X=%.1f Y=%.1f H=%.0f°", follower.getPose().getX(), follower.getPose().getY(), Math.toDegrees(follower.getPose().getHeading()));
        telemetry.addLine("---");
        telemetry.addData("Pose Match?", Math.abs(follower.getPose().getX() - startPose.getX()) < 1 ? "YES ✓" : "NO - PINPOINT NOT WORKING!");
        telemetry.addData("Paths Built", scorePreload != null ? "YES ✓" : "FAILED!");
        telemetry.update();
    }

    @Override
    public void start() {
        opmodeTimer.resetTimer();
        blocker.setPosition(0.3);
        shooter.setVelocity(getTickSpeed(SHOOTER_RPM));  // Start revving immediately
        setPathState(0);
    }

    /**
     * Update position using Limelight + Odometry sensor fusion
     * NOTE: Limelight uses field map (ftc2025DECODE.fmap) which returns positions
     * in field coordinates. We convert from Limelight's coordinate system to Pedro's.
     */
    private void updateLocalization() {
        double odoX = follower.getPose().getX();
        double odoY = follower.getPose().getY();
        
        limelightActive = false;
        tagIdSeen = -1;
        tagDistance = 0;
        
        LLResult result = limelight.getLatestResult();
        if (result != null && result.isValid()) {
            List<LLResultTypes.FiducialResult> fiducials = result.getFiducialResults();
            if (!fiducials.isEmpty()) {
                LLResultTypes.FiducialResult tag = fiducials.get(0);
                int tagId = (int) tag.getFiducialId();
                tagIdSeen = tagId;
                
                // Only use Red Goal tag (24) for position correction in auto
                if (tagId == 24) {
                    Pose3D robotPose = tag.getRobotPoseFieldSpace();
                    
                    // Limelight returns position in meters from field center
                    // Convert to Pedro coordinates (inches from top-left)
                    // Field is 141.24" x 141.24" (3.589m x 3.589m)
                    double fieldCenterX = 70.62;  // inches
                    double fieldCenterY = 70.62;  // inches
                    
                    // Limelight: +X forward (away from driver), +Y left, +Z up
                    // Pedro: +X right, +Y down from top-left corner
                    double llX = fieldCenterX - (robotPose.getPosition().y * 39.3701);  // LL +Y = Pedro -X
                    double llY = fieldCenterY - (robotPose.getPosition().x * 39.3701);  // LL +X = Pedro -Y
                    
                    tagDistance = Math.hypot(robotPose.getPosition().x, robotPose.getPosition().y) * 39.3701;
                    
                    // Sanity check: reject if correction is too large
                    double correctionDist = Math.hypot(llX - odoX, llY - odoY);
                    if (correctionDist < MAX_LIMELIGHT_CORRECTION) {
                        // Use Limelight position directly (100% trust when tag visible)
                        double correctedX = llX;
                        double correctedY = llY;
                        
                        // Boundary clamp
                        correctedX = Math.max(FIELD_MIN, Math.min(FIELD_MAX, correctedX));
                        correctedY = Math.max(FIELD_MIN, Math.min(FIELD_MAX, correctedY));
                        
                        // Update follower position
                        follower.setPose(new Pose(correctedX, correctedY, follower.getPose().getHeading()));
                        limelightActive = true;
                        limelightUpdateCount++;
                    }
                }
            }
        }
        
        lastOdoX = odoX;
        lastOdoY = odoY;
    }
    
    @Override
    public void loop() {
        // Emergency stop if auto period is ending
        if (opmodeTimer.getElapsedTimeSeconds() > MAX_AUTO_TIME) {
            shooter.setVelocity(0);
            intake.setPower(0);
            requestOpModeStop();
            return;
        }
        
        // Path timeout check - skip to next state if stuck too long
        if (follower.isBusy() && pathTimer.getElapsedTimeSeconds() > MAX_PATH_TIME) {
            telemetry.addData("WARNING", "Path timeout - skipping");
            follower.breakFollowing();
        }
        
        follower.update();
        updateLocalization();
        autonomousPathUpdate();

        // Telemetry
        telemetry.addData("Path State", pathState);
        telemetry.addData("Score State", scoreState);
        telemetry.addData("X", "%.1f\"", follower.getPose().getX());
        telemetry.addData("Y", "%.1f\"", follower.getPose().getY());
        telemetry.addData("Heading", "%.0f°", Math.toDegrees(follower.getPose().getHeading()));
        
        // Get target pose based on current path state
        Pose targetPose = null;
        String targetName = "None";
        if (pathState == 0 || pathState == 1) {
            targetPose = shootPose;
            targetName = "Shoot";
        } else if (pathState == 2) {
            targetPose = intakePose1;
            targetName = "Sample 1";
        } else if (pathState == 3) {
            targetPose = shootPose;
            targetName = "Shoot";
        } else if (pathState == 5) {
            targetPose = intakePose2Bridge;
            targetName = "Bridge 2";
        } else if (pathState == 6) {
            targetPose = intakePose2;
            targetName = "Sample 2";
        } else if (pathState == 9) {
            targetPose = intakePose3;
            targetName = "Bridge 3";
        } else if (pathState == 10) {
            targetPose = intakePose3b;
            targetName = "Sample 3";
        }
        
        if (targetPose != null) {
            double distToTarget = Math.hypot(
                follower.getPose().getX() - targetPose.getX(),
                follower.getPose().getY() - targetPose.getY()
            );
            telemetry.addData("Target", "%s (%.1f\" away)", targetName, distToTarget);
        }
        
        telemetry.addData("Shooter RPM", "%.0f", shooter.getVelocity() * 60 / TICKS_PER_REV);
        telemetry.addData("AprilTag", tagIdSeen >= 0 ? "#" + tagIdSeen + " (" + String.format("%.1f\"", tagDistance) + ")" : "None");
        telemetry.addData("LL Corrections", limelightUpdateCount);
        
        // Odometry health check
        double deltaX = Math.abs(follower.getPose().getX() - lastOdoX);
        double deltaY = Math.abs(follower.getPose().getY() - lastOdoY);
        boolean odoMoving = (deltaX > 0.1 || deltaY > 0.1);
        telemetry.addData("Odo Updating?", odoMoving ? "YES ✓" : "NO - FROZEN!");
        
        telemetry.addData("Time", "%.1fs", opmodeTimer.getElapsedTimeSeconds());
        telemetry.addData("Follower Busy", follower.isBusy());
        telemetry.update();
    }

    @Override
    public void stop() {
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
}
