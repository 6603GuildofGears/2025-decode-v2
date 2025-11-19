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

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

@Autonomous(name = "Red Close Auto", group = "Competition")
public class RedCloseAuto extends OpMode {

    private Follower follower;
    private Timer pathTimer, opmodeTimer, actionTimer;
    private int pathState;
    private int scoreState;  // Track scoring sequence

    // Hardware - Use DcMotorEx for all motors for consistency
    private DcMotorEx frontLeftDrive, frontRightDrive, backLeftDrive, backRightDrive;
    private DcMotorEx shooter;  // Note: in hardware config this is labeled "intakeMotor"
    private DcMotorEx intake;   // Note: in hardware config this is labeled "shooterMotor"
    private Servo blocker;

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

    // Poses for Red Close starting position
    private final Pose startPose = new Pose(9, 63.5, Math.toRadians(180)); // Red close start
    private final Pose scorePose = new Pose(14, 74, Math.toRadians(135)); // Facing basket
    private final Pose pickup1Pose = new Pose(23.5, 120, Math.toRadians(90)); // First sample
    private final Pose pickup2Pose = new Pose(23.5, 130, Math.toRadians(90)); // Second sample
    private final Pose pickup3Pose = new Pose(34, 120, Math.toRadians(135)); // Third sample
    private final Pose parkPose = new Pose(60, 98, Math.toRadians(90)); // Observation zone

    // Paths
    private Path scorePreload;
    private PathChain grabSample1, scoreSample1, grabSample2, scoreSample2, grabSample3, scoreSample3, park;

    /**
     * Build all autonomous paths
     */
    public void buildPaths() {
        // Score preloaded sample in high basket
        scorePreload = new Path(new BezierLine(startPose, scorePose));
        scorePreload.setLinearHeadingInterpolation(startPose.getHeading(), scorePose.getHeading());

        // Grab first sample from spike marks
        grabSample1 = follower.pathBuilder()
                .addPath(new BezierLine(scorePose, pickup1Pose))
                .setLinearHeadingInterpolation(scorePose.getHeading(), pickup1Pose.getHeading())
                .build();

        // Return to score first sample
        scoreSample1 = follower.pathBuilder()
                .addPath(new BezierLine(pickup1Pose, scorePose))
                .setLinearHeadingInterpolation(pickup1Pose.getHeading(), scorePose.getHeading())
                .build();

        // Grab second sample
        grabSample2 = follower.pathBuilder()
                .addPath(new BezierLine(scorePose, pickup2Pose))
                .setLinearHeadingInterpolation(scorePose.getHeading(), pickup2Pose.getHeading())
                .build();

        // Score second sample
        scoreSample2 = follower.pathBuilder()
                .addPath(new BezierLine(pickup2Pose, scorePose))
                .setLinearHeadingInterpolation(pickup2Pose.getHeading(), scorePose.getHeading())
                .build();

        // Grab third sample
        grabSample3 = follower.pathBuilder()
                .addPath(new BezierLine(scorePose, pickup3Pose))
                .setLinearHeadingInterpolation(scorePose.getHeading(), pickup3Pose.getHeading())
                .build();

        // Score third sample
        scoreSample3 = follower.pathBuilder()
                .addPath(new BezierLine(pickup3Pose, scorePose))
                .setLinearHeadingInterpolation(pickup3Pose.getHeading(), scorePose.getHeading())
                .build();

        // Park in observation zone
        park = follower.pathBuilder()
                .addPath(new BezierLine(scorePose, parkPose))
                .setLinearHeadingInterpolation(scorePose.getHeading(), parkPose.getHeading())
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
                    resetScoring();
                    setPathState(2);
                }
                scoreSequence();
                if (scoringComplete()) {
                    follower.followPath(grabSample1, true);
                    setPathState(3);
                }
                break;

            case 2: // Waiting for scoring to complete
                scoreSequence();
                break;

            case 3: // Grab first sample
                if (!follower.isBusy()) {
                    intake.setPower(-0.5);  // Run intake to grab
                    if (pathTimer.getElapsedTimeSeconds() > 1.0) {  // Grab for 1 second
                        intake.setPower(0);
                        follower.followPath(scoreSample1, true);
                        setPathState(4);
                    }
                }
                break;

            case 4: // Score first sample (2 samples)
                if (!follower.isBusy()) {
                    resetScoring();
                    setPathState(5);
                }
                scoreSequence();
                if (scoringComplete()) {
                    follower.followPath(grabSample2, true);
                    setPathState(6);
                }
                break;

            case 5: // Waiting for scoring to complete
                scoreSequence();
                break;

            case 6: // Grab second sample
                if (!follower.isBusy()) {
                    intake.setPower(-0.5);  // Run intake to grab
                    if (pathTimer.getElapsedTimeSeconds() > 1.0) {  // Grab for 1 second
                        intake.setPower(0);
                        follower.followPath(scoreSample2, true);
                        setPathState(7);
                    }
                }
                break;

            case 7: // Score second sample (2 samples)
                if (!follower.isBusy()) {
                    resetScoring();
                    setPathState(8);
                }
                scoreSequence();
                if (scoringComplete()) {
                    follower.followPath(grabSample3, true);
                    setPathState(9);
                }
                break;

            case 8: // Waiting for scoring to complete
                scoreSequence();
                break;

            case 9: // Grab third sample
                if (!follower.isBusy()) {
                    intake.setPower(-0.5);  // Run intake to grab
                    if (pathTimer.getElapsedTimeSeconds() > 1.0) {  // Grab for 1 second
                        intake.setPower(0);
                        follower.followPath(scoreSample3, true);
                        setPathState(10);
                    }
                }
                break;

            case 10: // Score third sample (2 samples)
                if (!follower.isBusy()) {
                    resetScoring();
                    setPathState(11);
                }
                scoreSequence();
                if (scoringComplete()) {
                    follower.followPath(park, true);
                    setPathState(12);
                }
                break;

            case 11: // Waiting for scoring to complete
                scoreSequence();
                break;

            case 12: // Park
                if (!follower.isBusy()) {
                    setPathState(-1); // Stop
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
        frontLeftDrive = hardwareMap.get(DcMotorEx.class, "frontLeftDrive");
        frontRightDrive = hardwareMap.get(DcMotorEx.class, "frontRightDrive");
        backLeftDrive = hardwareMap.get(DcMotorEx.class, "backLeftDrive");
        backRightDrive = hardwareMap.get(DcMotorEx.class, "backRightDrive");
        shooter = hardwareMap.get(DcMotorEx.class, "intakeMotor");  // Actual shooter motor
        intake = hardwareMap.get(DcMotorEx.class, "shooterMotor");  // Actual intake motor
        blocker = hardwareMap.get(Servo.class, "blocker");

        // Drivetrain setup (matches Rango configuration)
        frontLeftDrive.setDirection(DcMotorSimple.Direction.REVERSE);
        backLeftDrive.setDirection(DcMotorSimple.Direction.FORWARD);
        frontRightDrive.setDirection(DcMotorSimple.Direction.FORWARD);
        backRightDrive.setDirection(DcMotorSimple.Direction.FORWARD);

        frontLeftDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        frontRightDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backLeftDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backRightDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        // Shooter setup (this is the actual shooter, mapped to "intakeMotor")
        shooter.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        shooter.setDirection(DcMotorSimple.Direction.REVERSE);

        // Intake setup (this is the actual intake, mapped to "shooterMotor")
        intake.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        intake.setDirection(DcMotorSimple.Direction.FORWARD);

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

    @Override
    public void loop() {
        follower.update();
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
