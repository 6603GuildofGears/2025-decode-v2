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
@Autonomous(name = "Red - Far", group = "Competition")
public class BlueFarAuto extends OpMode {

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

    private final double TICKS_PER_REV = 28;
    private static final double FIELD_CENTER_X = 70.62;
    
    // Safety timeouts
    private static final double MAX_PATH_TIME = 8.0;
    private static final double MAX_AUTO_TIME = 29.5;
    private static final double MAX_LIMELIGHT_CORRECTION = 12.0;
    private static final double FIELD_MIN = 0.0;
    private static final double FIELD_MAX = 141.24;  // Half of 141.24
    
    // Scoring constants
    public static double SHOOTER_RPM = 3700;
    public static double INTAKE_PICKUP_POWER = 0.875;
    public static double INTAKE_FEED_POWER = 0.625;
    public static double RPM_TOLERANCE = 100;
    private static final double REV_UP_TIME = 3.0;
    private static final double PUSH_TIME = 2.5;
    private static final double PAUSE_TIME = 0.3;
    private static final double SETTLE_TIME = 0.5;
    private static final int SAMPLES_PER_SCORE = 2;
    private static final int PUSH_PULSES = 3;
    private int currentPulse = 0;
    
    // Servo positions
    private static final double BLOCKER_OPEN = 0.175;
    private static final double BLOCKER_CLOSED = 0.3;
    
    // AprilTag positions for localization (Blue alliance)
    private static final double[][] BLUE_TAG_POSITIONS = {
        {6.0, 47.25},      // Tag 14 - Blue Goal
        {6.0, 70.62},      // Tag 15 - Motif 11
        {6.0, 93.99},      // Tag 16 - Motif 12
        {6.0, 117.36}      // Tag 17 - Motif 13
    };

    // Poses for Blue Far starting position (mirrored from Red Far)
    // Red: (90, 9, 270°) -> Blue: mirror(90) = 51.24, heading = 180-270 = -90 = 270°
    public final Pose startPose = new Pose(mirror(90), 9, Math.toRadians(270));
    public final Pose shootPose = new Pose(mirror(75), 81, Math.toRadians(180 - 45));
    public final Pose intakePose1 = new Pose(mirror(130), 84, Math.toRadians(180 - 180));
    public final Pose intakePose2Bridge = new Pose(mirror(85), 60, Math.toRadians(180 - 135));
    public final Pose intakePose2 = new Pose(mirror(130), 59, Math.toRadians(180 - 180));
    public final Pose intakePose3Bridge = new Pose(mirror(85), 37, Math.toRadians(180 - 90));
    public final Pose intakePose3 = new Pose(mirror(130), 35, Math.toRadians(180 - 180));

    // Paths
    private Path scorePreload;
    private PathChain grabSample1, scoreSample1, grabSample2, grabSample2b, scoreSample2, grabSample3, grabSample3b, scoreSample3;

    /**
     * Mirror X coordinate for blue alliance
     */
    private double mirror(double x) {
        return 2 * FIELD_CENTER_X - x;
    }

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

        // Grab third sample - L-shaped path with bridge
        grabSample3 = follower.pathBuilder()
                .addPath(new BezierLine(shootPose, intakePose3Bridge))
                .setTangentHeadingInterpolation()
                .build();
        
        // Complete the L to sample 3
        grabSample3b = follower.pathBuilder()
                .addPath(new BezierLine(intakePose3Bridge, intakePose3))
                .setTangentHeadingInterpolation()
                .build();

        // Score third sample
        scoreSample3 = follower.pathBuilder()
                .addPath(new BezierLine(intakePose3, shootPose))
                .setTangentHeadingInterpolation()
                .build();
    }

    /**
     * Score samples into basket
     */
    private void scoreSequence() {
        switch (scoreState) {
            case 0: // Settle/re-aim before shooting
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
                
            case 2: // Wait for shooter to rev up
                if (actionTimer.getElapsedTimeSeconds() >= REV_UP_TIME) {
                    blocker.setPosition(BLOCKER_OPEN);
                    intake.setPower(INTAKE_FEED_POWER);
                    actionTimer.resetTimer();
                    scoreState++;
                }
                break;
                
            case 3: // Push first sample through
                if (actionTimer.getElapsedTimeSeconds() >= PUSH_TIME) {
                    intake.setPower(0);
                    blocker.setPosition(BLOCKER_CLOSED);
                    actionTimer.resetTimer();
                    scoreState++;
                }
                break;
                
            case 4: // Rev up for second sample
                if (actionTimer.getElapsedTimeSeconds() >= REV_UP_TIME) {
                    blocker.setPosition(BLOCKER_OPEN);
                    intake.setPower(INTAKE_FEED_POWER);
                    actionTimer.resetTimer();
                    scoreState++;
                }
                break;
                
            case 5: // Push second sample through
                if (actionTimer.getElapsedTimeSeconds() >= PUSH_TIME) {
                    intake.setPower(0);
                    blocker.setPosition(BLOCKER_CLOSED);
                    shooter.setVelocity(0);
                    scoreState++;
                }
                break;
                
            case 6: // Scoring complete
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
                        intake.setPower(INTAKE_PICKUP_POWER);
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
                if (!follower.isBusy()) {
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
                    intake.setPower(INTAKE_PICKUP_POWER);
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
                    intake.setPower(INTAKE_PICKUP_POWER);
                    follower.followPath(grabSample3, true);
                    setPathState(9);
                }
                break;

            case 9: // Move to bridge for third sample (intake running)
                if (!follower.isBusy()) {
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
                    setPathState(-1); // Done
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

        // Hardware mapping
        shooter = hardwareMap.get(DcMotorEx.class, "intakeMotor");
        intake = hardwareMap.get(DcMotorEx.class, "shooterMotor");
        blocker = hardwareMap.get(Servo.class, "blocker");

        shooter.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        shooter.setDirection(DcMotorSimple.Direction.REVERSE);

        intake.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        intake.setDirection(DcMotorSimple.Direction.REVERSE);
        
        // Limelight setup
        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.pipelineSwitch(0);
        limelight.start();

        buildPaths();

        telemetry.addLine("Red - Far Initialized");
        telemetry.update();
    }

    @Override
    public void start() {
        opmodeTimer.resetTimer();
        blocker.setPosition(BLOCKER_CLOSED);
        shooter.setVelocity(getTickSpeed(SHOOTER_RPM));
        setPathState(0);
    }

    /**
     * Update position using Limelight + Odometry sensor fusion
     */
    private void updateLocalization() {
        double odoX = follower.getPose().getX();
        double odoY = follower.getPose().getY();
        
        limelightActive = false;
        LLResult result = limelight.getLatestResult();
        if (result != null && result.isValid()) {
            List<LLResultTypes.FiducialResult> fiducials = result.getFiducialResults();
            if (!fiducials.isEmpty()) {
                LLResultTypes.FiducialResult tag = fiducials.get(0);
                int tagId = (int) tag.getFiducialId();
                
                if (tagId == 14) {
                    double[] tagFieldPos = BLUE_TAG_POSITIONS[0];
                    Pose3D robotPose = tag.getRobotPoseFieldSpace();
                    
                    double llX = tagFieldPos[0] - (robotPose.getPosition().x * 39.3701);
                    double llY = tagFieldPos[1] - (robotPose.getPosition().y * 39.3701);
                    
                    // Sanity check: reject if correction is too large
                    double correctionDist = Math.hypot(llX - odoX, llY - odoY);
                    if (correctionDist < MAX_LIMELIGHT_CORRECTION) {
                        double correctedX = (1.0 - LIMELIGHT_WEIGHT) * odoX + LIMELIGHT_WEIGHT * llX;
                        double correctedY = (1.0 - LIMELIGHT_WEIGHT) * odoY + LIMELIGHT_WEIGHT * llY;
                        
                        // Boundary clamp
                        correctedX = Math.max(FIELD_MIN, Math.min(FIELD_MAX, correctedX));
                        correctedY = Math.max(FIELD_MIN, Math.min(FIELD_MAX, correctedY));
                        
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
        
        // Path timeout check
        if (follower.isBusy() && pathTimer.getElapsedTimeSeconds() > MAX_PATH_TIME) {
            telemetry.addData("WARNING", "Path timeout - skipping");
            follower.breakFollowing();
        }
        
        follower.update();
        updateLocalization();
        autonomousPathUpdate();

        telemetry.addData("Path State", pathState);
        telemetry.addData("Score State", scoreState);
        telemetry.addData("X", "%.1f\"", follower.getPose().getX());
        telemetry.addData("Y", "%.1f\"", follower.getPose().getY());
        telemetry.addData("Heading", "%.0f°", Math.toDegrees(follower.getPose().getHeading()));
        telemetry.addData("Shooter RPM", "%.0f", shooter.getVelocity() * 60 / TICKS_PER_REV);
        telemetry.addData("Limelight", limelightActive ? "✓ (" + limelightUpdateCount + ")" : "✗");
        telemetry.addData("Time", "%.1fs", opmodeTimer.getElapsedTimeSeconds());
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
    
    private boolean isShooterAtSpeed() {
        double currentRPM = shooter.getVelocity() * 60 / TICKS_PER_REV;
        return Math.abs(currentRPM - SHOOTER_RPM) < RPM_TOLERANCE;
    }
}
