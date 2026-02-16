package org.firstinspires.ftc.teamcode.pedroPathing.Autos.Blue;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.pedropathing.util.Timer;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.*;
import com.pedropathing.paths.*;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.pedroPathing.other.Constants;

@Autonomous(name = "PEDRO - Blue Front Auto", group = "Blue")
public class Blue_Front extends OpMode {
    private Follower follower;
    private Timer pathTimer, opmodeTimer;

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

    // Poses from the provided coordinates
    private final Pose startPose = new Pose(21, 123, Math.toRadians(142));
    private final Pose shootPose = new Pose(49, 84, Math.toRadians(180));
    private final Pose intake1 = new Pose(24, 84, Math.toRadians(180));      // Intake 1 position
    private final Pose intakePose2 = new Pose(42, 60, Math.toRadians(180));  // Drive to intake 2
    private final Pose intake2 = new Pose(24, 60, Math.toRadians(180));      // Intake 2 position
    private final Pose endPose = new Pose(24, 72, Math.toRadians(180));      // Park position

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
                        // Open blocker and run intake
                    }

                    if (shooterTimer.seconds() >= 7) {
                        pathState = PathState.DRIVE_SHOOTPOSE_TO_INTAKE1;
                        shooterStarted = false;
                    }

                    telemetry.addLine("Preload Shot");
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
                        // Open blocker and run intake
                    }

                    if (shooterTimer.seconds() >= 8) {
                        pathState = PathState.DRIVE_SHOOTPOSE_TO_INTAKEPOSE2;
                        shooterStarted = false;
                    }

                    telemetry.addLine("Sample 1 Shot");
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
                        // Open blocker and run intake
                    }

                    telemetry.addLine("Sample 2 Shot");

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

    public void statePathUpdate(PathState newState) {
        pathState = newState;
        pathTimer.resetTimer();
    }

    @Override
    public void init() {
        pathState = PathState.DRIVE_STARTPOSE_TO_SHOOTPOSE;
        pathTimer = new Timer();
        opmodeTimer = new Timer();
        opmodeTimer.resetTimer();
        follower = Constants.createFollower(hardwareMap);

        buildPaths();
        follower.setPose(startPose);
    }

    public void start() {
        opmodeTimer.resetTimer();
        pathState = PathState.DRIVE_STARTPOSE_TO_SHOOTPOSE;
    }

    @Override
    public void loop() {
        follower.update();

        statePathUpdate();

        telemetry.addData("Path state", pathState.toString());
        telemetry.addData("X", follower.getPose().getX());
        telemetry.addData("Y", follower.getPose().getY());
        telemetry.addData("Heading", Math.toDegrees(follower.getPose().getHeading()));
        telemetry.addData("Path time (s): ", pathTimer.getElapsedTimeSeconds());
        telemetry.update();
    }
}
