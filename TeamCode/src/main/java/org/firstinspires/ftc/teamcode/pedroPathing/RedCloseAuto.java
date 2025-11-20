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

@Autonomous(name = "Red Close Auto", group = "Competition")
public class RedCloseAuto extends OpMode {

    private Follower follower;
    private Timer pathTimer, opmodeTimer, actionTimer;
    private int pathState;
    private int scoreState;  // Track scoring sequence

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

    private final double TICKS_PER_REV = 28;
    
    // Scoring constants
    public static double SHOOTER_RPM = 3700;  // Shooter speed from Rango
    public static double INTAKE_PUSH_POWER = 1.0;  // Power to push sample from Rango
    private static final double REV_UP_TIME = 3.0;  // Seconds to rev up shooter
    private static final double PUSH_TIME = 3.0;  // Seconds to push sample through (changed from 0.3)
    private static final double SETTLE_TIME = 0.5;  // Stop to re-aim before shooting
    private static final int SAMPLES_PER_SCORE = 2;  // Score 2 samples each time
    
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
    public final Pose intakePose1 = new Pose(130, 84, Math.toRadians(0));  // First sample
    public final Pose intakePose2 = new Pose(130, 55, Math.toRadians(0));  // Second sample
    public final Pose intakePose3 = new Pose(85, 37, Math.toRadians(0));  // Third sample (closer to submersible)
    public final Pose intakePose3b = new Pose(130, 35, Math.toRadians(0));  // Third sample pickup from other side

    // Paths
    private Path scorePreload;
    private PathChain grabSample1, scoreSample1, grabSample2, scoreSample2, grabSample3, grabSample3b, scoreSample3;

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

        // Grab second sample
        grabSample2 = follower.pathBuilder()
                .addPath(new BezierLine(shootPose, intakePose2))
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
                    intake.setPower(INTAKE_PUSH_POWER);
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
                    intake.setPower(INTAKE_PUSH_POWER);
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
     * Check if scoring sequence is complete
     */
    private boolean scoringComplete() {
        return scoreState >= 6;
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
            case 0: // Start - drive to basket
                follower.followPath(scorePreload);
                setPathState(1);
                break;

            case 1: // Score preload (2 samples)
                if (!follower.isBusy()) {
                    scoreSequence();
                    if (scoringComplete()) {
                        resetScoring();
                        follower.followPath(grabSample1, true);
                        setPathState(2);
                    }
                }
                break;

            case 2: // Grab first sample
                if (!follower.isBusy()) {
                    intake.setPower(0.5);  // Run intake to grab
                    if (pathTimer.getElapsedTimeSeconds() > 1.0) {  // Grab for 1 second
                        intake.setPower(0);
                        shooter.setVelocity(getTickSpeed(SHOOTER_RPM));  // Start revving while driving
                        follower.followPath(scoreSample1, true);
                        setPathState(3);
                    }
                }
                break;

            case 3: // Score first sample (shooter already revved)
                if (!follower.isBusy()) {
                    scoreState = 2;  // Skip settle and rev - already done
                    scoreSequence();
                    if (scoringComplete()) {
                        resetScoring();
                        follower.followPath(grabSample2, true);
                        setPathState(4);
                    }
                }
                break;

            case 4: // Grab second sample
                if (!follower.isBusy()) {
                    intake.setPower(0.5);  // Run intake to grab
                    if (pathTimer.getElapsedTimeSeconds() > 1.0) {  // Grab for 1 second
                        intake.setPower(0);
                        shooter.setVelocity(getTickSpeed(SHOOTER_RPM));  // Start revving while driving
                        follower.followPath(scoreSample2, true);
                        setPathState(5);
                    }
                }
                break;

            case 5: // Score second sample (shooter already revved)
                if (!follower.isBusy()) {
                    scoreState = 2;  // Skip settle and rev - already done
                    scoreSequence();
                    if (scoringComplete()) {
                        resetScoring();
                        follower.followPath(grabSample3, true);
                        setPathState(6);
                    }
                }
                break;

            case 6: // Move to intermediate position for third sample
                if (!follower.isBusy()) {
                    follower.followPath(grabSample3b, true);
                    setPathState(7);
                }
                break;

            case 7: // Grab third sample
                if (!follower.isBusy()) {
                    intake.setPower(0.5);  // Run intake to grab
                    if (pathTimer.getElapsedTimeSeconds() > 1.0) {  // Grab for 1 second
                        intake.setPower(0);
                        shooter.setVelocity(getTickSpeed(SHOOTER_RPM));  // Start revving while driving
                        follower.followPath(scoreSample3, true);
                        setPathState(8);
                    }
                }
                break;

            case 8: // Score third sample (shooter already revved)
                if (!follower.isBusy()) {
                    scoreState = 2;  // Skip settle and rev - already done
                    scoreSequence();
                    if (scoringComplete()) {
                        setPathState(-1); // Done - no parking
                    }
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
        
        // Limelight setup
        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.pipelineSwitch(0);
        limelight.start();

        // Build paths
        buildPaths();

        telemetry.addLine("Red Close Auto Initialized");
        telemetry.addLine("NOTE: Shooter is 'intakeMotor', Intake is 'shooterMotor'");
        telemetry.update();
    }

    @Override
    public void start() {
        opmodeTimer.resetTimer();
        blocker.setPosition(0.3);
        setPathState(0);
    }

    /**
     * Update position using Limelight + Odometry sensor fusion
     */
    private void updateLocalization() {
        double odoX = follower.getPose().getX();
        double odoY = follower.getPose().getY();
        
        LLResult result = limelight.getLatestResult();
        if (result != null && result.isValid()) {
            List<LLResultTypes.FiducialResult> fiducials = result.getFiducialResults();
            if (!fiducials.isEmpty()) {
                LLResultTypes.FiducialResult tag = fiducials.get(0);
                int tagId = (int) tag.getFiducialId();
                
                // Only use Red Goal tag (24) for position correction in auto
                if (tagId == 24) {
                    double[] tagFieldPos = RED_TAG_POSITIONS[0];  // Tag 24
                    Pose3D robotPose = tag.getRobotPoseFieldSpace();
                    
                    // Calculate corrected position
                    double llX = tagFieldPos[0] + (robotPose.getPosition().x * 39.3701);
                    double llY = tagFieldPos[1] - (robotPose.getPosition().y * 39.3701);
                    
                    // Blend with odometry
                    double correctedX = (1.0 - LIMELIGHT_WEIGHT) * odoX + LIMELIGHT_WEIGHT * llX;
                    double correctedY = (1.0 - LIMELIGHT_WEIGHT) * odoY + LIMELIGHT_WEIGHT * llY;
                    
                    // Update follower position
                    follower.setPose(new Pose(correctedX, correctedY, follower.getPose().getHeading()));
                }
            }
        }
        
        lastOdoX = odoX;
        lastOdoY = odoY;
    }
    
    @Override
    public void loop() {
        follower.update();
        updateLocalization();
        autonomousPathUpdate();

        // Telemetry
        telemetry.addData("Path State", pathState);
        telemetry.addData("Score State", scoreState);
        telemetry.addData("X", follower.getPose().getX());
        telemetry.addData("Y", follower.getPose().getY());
        telemetry.addData("Heading", Math.toDegrees(follower.getPose().getHeading()));
        telemetry.addData("Shooter RPM", shooter.getVelocity() * 60 / TICKS_PER_REV);
        telemetry.addData("Blocker", blocker.getPosition());
        telemetry.update();
    }

    @Override
    public void stop() {
    }

    /**
     * Utility: Convert RPM to ticks per second
     */
    public double getTickSpeed(double rpm) {
        return rpm * TICKS_PER_REV / 60;
    }
}
