package org.firstinspires.ftc.teamcode.pedroPathing.Autos.Motif;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.pedropathing.util.Timer;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.*;
import com.pedropathing.paths.*;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.pedroPathing.other.Constants;
import org.firstinspires.ftc.teamcode.pedroPathing.Pipelines.Limelight_Pipeline;
import org.firstinspires.ftc.teamcode.pedroPathing.Pipelines.Motif;
import org.firstinspires.ftc.teamcode.pedroPathing.Pipelines.SpindexerController;
import org.firstinspires.ftc.teamcode.pedroPathing.Pipelines.Servo_Pipeline;
import org.firstinspires.ftc.teamcode.pedroPathing.Pipelines.Sensor;

/**
 * Blue Front Auto with Motif detection.
 * Scans for obelisk tag (21/22/23) during init_loop,
 * then shoots balls in the motif-required color order.
 */
@Disabled
@Autonomous(name = "Blue Front MOTIF", group = "Blue")
public class Blue_Front_Motif extends OpMode {
    private Follower follower;
    private Timer pathTimer, opmodeTimer;
    private SpindexerController spindexer;
    private DcMotorEx flywheel;

    // Motif detection
    private int detectedTagId = -1;
    private String[] motifOrder = null;
    private boolean motifLocked = false;

    public enum PathState {
        DRIVE_STARTPOSE_TO_SHOOTPOSE,
        SHOOT_PRELOAD,
        DRIVE_SHOOTPOSE_TO_INTAKE1,
        DRIVE_INTAKE1_TO_SHOOTPOSE,
        SHOOT_INTAKE1,
        DRIVE_SHOOTPOSE_TO_INTAKEPOSE2,
        DRIVE_INTAKEPOSE2_TO_INTAKE2,
        DRIVE_INTAKE2_TO_SHOOTPOSE,
        SHOOT_INTAKE2,
        DRIVE_SHOOTPOSE_TO_ENDPOSE,
    }

    PathState pathState;

    // Poses (same as Blue_Front)
    private final Pose startPose = new Pose(21, 123, Math.toRadians(142));
    private final Pose shootPose = new Pose(49, 84, Math.toRadians(180));
    private final Pose intake1 = new Pose(24, 78, Math.toRadians(180));
    private final Pose intakePose2 = new Pose(42, 52, Math.toRadians(180));
    private final Pose intake2 = new Pose(24, 52, Math.toRadians(180));
    private final Pose endPose = new Pose(24, 72, Math.toRadians(180));

    private ElapsedTime shooterTimer = new ElapsedTime();
    private boolean shooterStarted = false;
    private boolean pathStarted = false;

    private PathChain driveStartPoseToShootPose;
    private PathChain driveShootPoseToIntake1;
    private PathChain driveIntake1ToShootPose;
    private PathChain driveShootPoseToIntakePose2;
    private PathChain driveIntakePose2ToIntake2;
    private PathChain driveIntake2ToShootPose;
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

        driveIntake1ToShootPose = follower.pathBuilder()
                .addPath(new BezierLine(intake1, shootPose))
                .setLinearHeadingInterpolation(intake1.getHeading(), shootPose.getHeading())
                .build();

        driveShootPoseToIntakePose2 = follower.pathBuilder()
                .addPath(new BezierLine(shootPose, intakePose2))
                .setConstantHeadingInterpolation(intakePose2.getHeading())
                .build();

        driveIntakePose2ToIntake2 = follower.pathBuilder()
                .addPath(new BezierLine(intakePose2, intake2))
                .setConstantHeadingInterpolation(intake2.getHeading())
                .build();

        driveIntake2ToShootPose = follower.pathBuilder()
                .addPath(new BezierLine(intake2, shootPose))
                .setLinearHeadingInterpolation(intake2.getHeading(), shootPose.getHeading())
                .build();

        driveShootPoseToEndPose = follower.pathBuilder()
                .addPath(new BezierLine(shootPose, endPose))
                .setConstantHeadingInterpolation(endPose.getHeading())
                .build();
    }

    public void statePathUpdate() {
        switch (pathState) {
            case DRIVE_STARTPOSE_TO_SHOOTPOSE:
                if (!pathStarted) {
                    follower.followPath(driveStartPoseToShootPose, true);
                    pathStarted = true;
                }
                if (follower.isBusy()) {
                    follower.update();
                } else {
                    pathState = PathState.SHOOT_PRELOAD;
                    pathStarted = false;
                    shooterTimer.reset();
                }
                break;

            case SHOOT_PRELOAD:
                if (!follower.isBusy()) {
                    if (shooterTimer.seconds() >= 2.0) {
                        // Shoot preloaded balls in motif order
                    }

                    if (shooterTimer.seconds() >= 7) {
                        pathState = PathState.DRIVE_SHOOTPOSE_TO_INTAKE1;
                        shooterStarted = false;
                    }

                    telemetry.addLine("Preload Shot — Motif: " + Motif.getMotifName(detectedTagId));
                }
                break;

            case DRIVE_SHOOTPOSE_TO_INTAKE1:
                if (!pathStarted) {
                    follower.followPath(driveShootPoseToIntake1, true);
                    pathStarted = true;
                }
                if (pathStarted && !follower.isBusy()) {
                    pathState = PathState.DRIVE_INTAKE1_TO_SHOOTPOSE;
                    pathStarted = false;
                }
                break;

            case DRIVE_INTAKE1_TO_SHOOTPOSE:
                if (!pathStarted) {
                    follower.followPath(driveIntake1ToShootPose, true);
                    pathStarted = true;
                }
                if (follower.getCurrentTValue() >= 0.5 && !shooterStarted) {
                    shooterTimer.reset();
                    shooterStarted = true;
                }
                if (!follower.isBusy()) {
                    pathState = PathState.SHOOT_INTAKE1;
                    pathStarted = false;
                }
                break;

            case SHOOT_INTAKE1:
                if (!follower.isBusy()) {
                    if (shooterTimer.seconds() >= 2.0) {
                        // Shoot with motif order
                    }
                    if (shooterTimer.seconds() >= 8) {
                        pathState = PathState.DRIVE_SHOOTPOSE_TO_INTAKEPOSE2;
                        shooterStarted = false;
                    }
                    telemetry.addLine("Sample 1 Shot — Motif: " + Motif.getMotifName(detectedTagId));
                }
                break;

            case DRIVE_SHOOTPOSE_TO_INTAKEPOSE2:
                if (!pathStarted) {
                    follower.followPath(driveShootPoseToIntakePose2, true);
                    pathStarted = true;
                }
                if (pathStarted && !follower.isBusy()) {
                    pathState = PathState.DRIVE_INTAKEPOSE2_TO_INTAKE2;
                    pathStarted = false;
                }
                break;

            case DRIVE_INTAKEPOSE2_TO_INTAKE2:
                if (!pathStarted) {
                    follower.followPath(driveIntakePose2ToIntake2, true);
                    pathStarted = true;
                }
                if (pathStarted && !follower.isBusy()) {
                    pathState = PathState.DRIVE_INTAKE2_TO_SHOOTPOSE;
                    pathStarted = false;
                }
                break;

            case DRIVE_INTAKE2_TO_SHOOTPOSE:
                if (!pathStarted) {
                    follower.followPath(driveIntake2ToShootPose, true);
                    pathStarted = true;
                }
                if (follower.getCurrentTValue() >= 0.5 && !shooterStarted) {
                    shooterTimer.reset();
                    shooterStarted = true;
                }
                if (!follower.isBusy()) {
                    pathState = PathState.SHOOT_INTAKE2;
                    pathStarted = false;
                }
                break;

            case SHOOT_INTAKE2:
                if (!follower.isBusy()) {
                    if (shooterTimer.seconds() >= 2.0) {
                        // Shoot with motif order
                    }
                    telemetry.addLine("Sample 2 Shot — Motif: " + Motif.getMotifName(detectedTagId));
                    if (shooterTimer.seconds() >= 6) {
                        pathState = PathState.DRIVE_SHOOTPOSE_TO_ENDPOSE;
                        shooterStarted = false;
                    }
                }
                break;

            case DRIVE_SHOOTPOSE_TO_ENDPOSE:
                if (!pathStarted) {
                    follower.followPath(driveShootPoseToEndPose, true);
                    pathStarted = true;
                }
                if (pathStarted && !follower.isBusy()) {
                    telemetry.addLine("Auto Complete");
                    pathStarted = false;
                    requestOpModeStop();
                }
                break;

            default:
                telemetry.addLine("No valid path state");
                break;
        }
    }

    @Override
    public void init() {
        pathState = PathState.DRIVE_STARTPOSE_TO_SHOOTPOSE;
        pathTimer = new Timer();
        opmodeTimer = new Timer();
        opmodeTimer.resetTimer();
        follower = Constants.createFollower(hardwareMap);

        // Initialize Limelight for obelisk scanning
        Limelight_Pipeline.initLimelight(this);

        // Initialize spindexer
        spindexer = new SpindexerController();

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
        opmodeTimer.resetTimer();
        pathState = PathState.DRIVE_STARTPOSE_TO_SHOOTPOSE;

        // Apply motif to spindexer (or use default order if not detected)
        if (motifLocked && motifOrder != null) {
            spindexer.setMotifOrder(motifOrder);
        }
    }

    @Override
    public void loop() {
        follower.update();
        statePathUpdate();

        telemetry.addData("Motif", motifLocked ? Motif.getMotifName(detectedTagId) : "NONE (default order)");
        telemetry.addData("Path state", pathState.toString());
        telemetry.addData("X", follower.getPose().getX());
        telemetry.addData("Y", follower.getPose().getY());
        telemetry.addData("Heading", Math.toDegrees(follower.getPose().getHeading()));
        telemetry.addData("Path time (s): ", pathTimer.getElapsedTimeSeconds());
        telemetry.update();
    }
}
