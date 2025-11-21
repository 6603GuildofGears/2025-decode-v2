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

/**
 * Red Alliance Far Starting Position Autonomous
 * Starts from far tile (near top wall), drives to basket, shoots 3 preloads
 */
@Disabled
@Autonomous(name = "Red Far Auto", group = "Red")
public class RedFarAuto extends OpMode {

    private Follower follower;
    private Timer pathTimer, opmodeTimer, actionTimer;
    private int pathState;
    private int scoreState;

    // Hardware
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
    
    // Safety timeouts
    private static final double MAX_PATH_TIME = 8.0;
    private static final double MAX_AUTO_TIME = 29.5;
    private static final double MAX_LIMELIGHT_CORRECTION = 12.0;
    private static final double FIELD_MIN = 0.0;
    private static final double FIELD_MAX = 141.24;
    
    // Scoring constants
    public static double SHOOTER_RPM = 3700;
    public static double INTAKE_PUSH_POWER = 1.0;
    private static final double REV_UP_TIME = 3.0;
    private static final double PUSH_TIME = 3.0;
    private static final double SETTLE_TIME = 0.5;
    
    // Servo positions
    private static final double BLOCKER_OPEN = 0.175;
    private static final double BLOCKER_CLOSED = 0.3;
    
    // AprilTag positions for localization (Red alliance)
    private static final double[][] RED_TAG_POSITIONS = {
        {135.24, 47.25},   // Tag 24 - Red Goal
        {135.24, 70.62},   // Tag 25 - Motif 21
        {135.24, 93.99},   // Tag 26 - Motif 22
        {135.24, 117.36}   // Tag 27 - Motif 23
    };

    // Poses - far starting position (Net Zone - red wall)
    public final Pose startPose = new Pose(135, 85, Math.toRadians(180));  // Red wall, facing toward field
    public final Pose shootPose = new Pose(110, 25, Math.toRadians(45));  // Drive to shooting position near red processor

    // Path
    private Path driveToShoot;

    /**
     * Build the path to shooting position
     */
    public void buildPaths() {
        driveToShoot = new Path(new BezierLine(startPose, shootPose));
        driveToShoot.setLinearHeadingInterpolation(startPose.getHeading(), shootPose.getHeading());
    }

    /**
     * Score samples into basket
     */
    private void scoreSequence() {
        switch (scoreState) {
            case 0: // Settle
                if (actionTimer.getElapsedTimeSeconds() > SETTLE_TIME) {
                    actionTimer.resetTimer();
                    scoreState = 1;
                }
                break;

            case 1: // Rev up shooter
                shooter.setVelocity(getTickSpeed(SHOOTER_RPM));
                if (actionTimer.getElapsedTimeSeconds() > REV_UP_TIME) {
                    actionTimer.resetTimer();
                    scoreState = 2;
                }
                break;

            case 2: // Open blocker
                blocker.setPosition(BLOCKER_OPEN);
                if (actionTimer.getElapsedTimeSeconds() > 0.3) {
                    actionTimer.resetTimer();
                    scoreState = 3;
                }
                break;

            case 3: // Push samples through
                intake.setPower(INTAKE_PUSH_POWER);
                if (actionTimer.getElapsedTimeSeconds() > PUSH_TIME) {
                    intake.setPower(0);
                    shooter.setVelocity(0);
                    blocker.setPosition(BLOCKER_CLOSED);
                    scoreState = 4;
                }
                break;

            case 4: // Done
                break;
        }
    }

    /**
     * Check if scoring is complete
     */
    private boolean scoringComplete() {
        return scoreState >= 4;
    }

    /**
     * Main autonomous state machine
     */
    public void autonomousPathUpdate() {
        switch (pathState) {
            case 0: // Drive to basket
                follower.followPath(driveToShoot);
                setPathState(1);
                break;

            case 1: // Wait to reach position
                if (!follower.isBusy()) {
                    setPathState(2);
                }
                break;

            case 2: // Score all 3 preloaded samples
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

        // Hardware mapping - NOTE: shooter/intake are swapped in hardware config
        shooter = hardwareMap.get(DcMotorEx.class, "intakeMotor");  // Actual shooter motor
        intake = hardwareMap.get(DcMotorEx.class, "shooterMotor");  // Actual intake motor
        blocker = hardwareMap.get(Servo.class, "blocker");

        // Shooter setup
        shooter.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        shooter.setDirection(DcMotorSimple.Direction.REVERSE);

        // Intake setup
        intake.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        intake.setDirection(DcMotorSimple.Direction.REVERSE);
        
        // Limelight setup
        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.pipelineSwitch(0);
        limelight.start();

        // Build paths
        buildPaths();

        telemetry.addLine("Blue - Far Initialized");
        telemetry.addLine("Starts from far tile, drives to basket, shoots 3 preloads");
        telemetry.update();
    }

    @Override
    public void start() {
        opmodeTimer.resetTimer();
        blocker.setPosition(BLOCKER_CLOSED);
        scoreState = 0;
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
                
                // Only use Red Goal tag (24) for position correction in auto
                if (tagId == 24) {
                    double[] tagFieldPos = RED_TAG_POSITIONS[0];  // Tag 24
                    Pose3D robotPose = tag.getRobotPoseFieldSpace();
                    
                    // Calculate corrected position
                    double llX = tagFieldPos[0] + (robotPose.getPosition().x * 39.3701);
                    double llY = tagFieldPos[1] - (robotPose.getPosition().y * 39.3701);
                    
                    // Sanity check: reject if correction is too large
                    double correctionDist = Math.hypot(llX - odoX, llY - odoY);
                    if (correctionDist < MAX_LIMELIGHT_CORRECTION) {
                        // Blend with odometry
                        double correctedX = (1.0 - LIMELIGHT_WEIGHT) * odoX + LIMELIGHT_WEIGHT * llX;
                        double correctedY = (1.0 - LIMELIGHT_WEIGHT) * odoY + LIMELIGHT_WEIGHT * llY;
                        
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
        
        // Path timeout check
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
        telemetry.addData("Shooter RPM", "%.0f", shooter.getVelocity() * 60 / TICKS_PER_REV);
        telemetry.addData("Limelight", limelightActive ? "✓ (" + limelightUpdateCount + ")" : "✗");
        telemetry.addData("Time", "%.1fs", opmodeTimer.getElapsedTimeSeconds());
        telemetry.update();
    }

    @Override
    public void stop() {
        shooter.setVelocity(0);
        intake.setPower(0);
    }

    /**
     * Utility: Convert RPM to ticks per second
     */
    public double getTickSpeed(double rpm) {
        return rpm * TICKS_PER_REV / 60;
    }
}
