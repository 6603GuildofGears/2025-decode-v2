package org.firstinspires.ftc.teamcode.pedroPathing.Autos.Red;

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
import org.firstinspires.ftc.teamcode.pedroPathing.Pipelines.Motor_PipeLine;

@Autonomous(name = "PEDRO - Red Back Auto", group = "Red")
public class Red_Back extends OpMode {
    private Follower follower;
    private Timer pathTimer, opmodeTimer;

    private DcMotorEx flywheel;
    private DcMotorEx intake;
    private Servo spindexer;
    private Motor_PipeLine motorPipeline;


        public enum PathState {
            DRIVE_STARTPOSE_TO_SHOOTPOSE,
            SHOOT_PRELOAD,
            DRIVE_SHOOTPOSE_TO_INTAKE1,
            DRIVE_INTAKE1_TO_INTAKE1POSE2,
            DRIVE_INTAKE1POSE2_TO_INTAKE1POSE3,
            DRIVE_INTAKE1POSE3_TO_SHOOTPOSE,
            SHOOT_INTAKE1,
            DRIVE_SHOOTPOSE_TO_INTAKE2,
            DRIVE_INTAKE2_TO_INTAKE2POSE1,
            DRIVE_INTAKE2POSE1_TO_INTAKE2POSE2,
            DRIVE_INTAKE2POSE2_TO_INTAKE2POSE3,
            DRIVE_INTAKE2POSE3_TO_SHOOTPOSE,
            SHOOT_INTAKE2,
            DRIVE_SHOOTPOSE_TO_ENDPOSE,
        }

              PathState pathState;


    private final Pose startPose = new Pose(84, 9, Math.toRadians(0));

    private final Pose shootPose = new Pose(84, 20, Math.toRadians(0));    // Shooting position

    private final Pose intake1 = new Pose(106, 36, Math.toRadians(0));       // Intake 1 position
    private final Pose intake1Pose2 = new Pose(109, 36, Math.toRadians(0));   // Intake 1 waypoint 2
    private final Pose intake1Pose3 = new Pose(112, 36, Math.toRadians(0));   // Intake 1 waypoint 3

    private final Pose intake2 = new Pose(135, 9.5, Math.toRadians(0));      // Intake 2 position
    private final Pose intake2Pose1 = new Pose(138, 9.5, Math.toRadians(0));  // Intake 2 waypoint 1
    private final Pose intake2Pose2 = new Pose(141, 9.5, Math.toRadians(0));  // Intake 2 waypoint 2
    private final Pose intake2Pose3 = new Pose(143, 9.5, Math.toRadians(0));  // Intake 2 waypoint 3

    private final Pose endPose = new Pose(120, 36, Math.toRadians(0));  // End position
   // Shooter hardware
   
    private ElapsedTime shooterTimer = new ElapsedTime();
    private boolean shooterStarted = false;
    private boolean pathStarted = false;


    private PathChain driveStartPoseShootPose;
    private PathChain driveShootPoseToIntake1;
    private PathChain driveIntake1ToIntake1Pose2;
    private PathChain driveIntake1Pose2ToIntake1Pose3;
    private PathChain driveIntake1Pose3ToShootPose;
    private PathChain driveShootPoseToIntake2;
    private PathChain driveIntake2ToIntake2Pose1;
    private PathChain driveIntake2Pose1ToIntake2Pose2;
    private PathChain driveIntake2Pose2ToIntake2Pose3;
    private PathChain driveIntake2Pose3ToShootPose;
    private PathChain driveShootPoseToEndPose;


 public void buildPaths() {

     driveStartPoseShootPose = follower.pathBuilder()
            .addPath(new BezierLine(startPose, shootPose))
            .setConstantHeadingInterpolation(shootPose.getHeading())
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

        driveIntake1Pose3ToShootPose = follower.pathBuilder()
            .addPath(new BezierLine(intake1Pose3, shootPose))
            .setLinearHeadingInterpolation(intake1Pose3.getHeading(), shootPose.getHeading())
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

     public void statePathUpdate() {
        switch (pathState) {
            case DRIVE_STARTPOSE_TO_SHOOTPOSE:
                if (!pathStarted) {
                    follower.followPath(driveStartPoseShootPose, true);
                    pathStarted = true;
                }
                if (pathStarted && !follower.isBusy()) {
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
                    follower.followPath(driveShootPoseToIntake1, 0.375, true);
                    pathStarted = true;

                    intake.setPower(-0.4);
                }

                if (pathStarted && !follower.isBusy()) {
                    pathState = PathState.DRIVE_INTAKE1_TO_INTAKE1POSE2;
                    pathStarted = false;
                }
                break;

            case DRIVE_INTAKE1_TO_INTAKE1POSE2:
                if (!pathStarted) {
                    follower.followPath(driveIntake1ToIntake1Pose2, 0.375, true);
                    pathStarted = true;

                    intake.setPower(-0.4);
                }

                if (pathStarted && !follower.isBusy()) {
                    pathState = PathState.DRIVE_INTAKE1POSE2_TO_INTAKE1POSE3;
                    pathStarted = false;
                }
                break;

            case DRIVE_INTAKE1POSE2_TO_INTAKE1POSE3:
                if (!pathStarted) {
                    follower.followPath(driveIntake1Pose2ToIntake1Pose3, 0.375, true);
                    pathStarted = true;

                    intake.setPower(-0.4);
                }

                if (pathStarted && !follower.isBusy()) {
                    pathState = PathState.DRIVE_INTAKE1POSE3_TO_SHOOTPOSE;
                    pathStarted = false;
                }
                break;

            case DRIVE_INTAKE1POSE3_TO_SHOOTPOSE:
                if (!pathStarted) {
                    follower.followPath(driveIntake1Pose3ToShootPose, true);
                    pathStarted = true;
                }

                if (follower.getCurrentTValue() >= 0.5) {
                    intake.setPower(0);
                }

                if (pathStarted && !follower.isBusy()) {
                    pathState = PathState.SHOOT_INTAKE1;
                    pathStarted = false;
                    shooterTimer.reset();
                }
                break;

            case SHOOT_INTAKE1:
                if (!follower.isBusy()) {
                    if (shooterTimer.seconds() >= 2.0) {
                        // Open blocker and run intake
                    }

                    if (shooterTimer.seconds() >= 8) {
                        pathState = PathState.DRIVE_SHOOTPOSE_TO_INTAKE2;
                        shooterStarted = false;
                    }

                    telemetry.addLine("Sample 1 Shot");
                }
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
                    follower.followPath(driveIntake2ToIntake2Pose1, 0.375, true);
                    pathStarted = true;

                    intake.setPower(-0.4);
                }

                if (pathStarted && !follower.isBusy()) {
                    pathState = PathState.DRIVE_INTAKE2POSE1_TO_INTAKE2POSE2;
                    pathStarted = false;
                }
                break;

            case DRIVE_INTAKE2POSE1_TO_INTAKE2POSE2:
                if (!pathStarted) {
                    follower.followPath(driveIntake2Pose1ToIntake2Pose2, 0.375, true);
                    pathStarted = true;

                    intake.setPower(-0.4);
                }

                if (pathStarted && !follower.isBusy()) {
                    pathState = PathState.DRIVE_INTAKE2POSE2_TO_INTAKE2POSE3;
                    pathStarted = false;
                }
                break;

            case DRIVE_INTAKE2POSE2_TO_INTAKE2POSE3:
                if (!pathStarted) {
                    follower.followPath(driveIntake2Pose2ToIntake2Pose3, 0.375, true);
                    pathStarted = true;

                    intake.setPower(-0.4);
                }

                if (pathStarted && !follower.isBusy()) {
                    pathState = PathState.DRIVE_INTAKE2POSE3_TO_SHOOTPOSE;
                    pathStarted = false;
                }
                break;

            case DRIVE_INTAKE2POSE3_TO_SHOOTPOSE:
                if (!pathStarted) {
                    follower.followPath(driveIntake2Pose3ToShootPose, true);
                    pathStarted = true;
                }

                if (follower.getCurrentTValue() >= 0.5) {
                    intake.setPower(0);
                }

                if (follower.getCurrentTValue() >= 0.5 && !shooterStarted) {
                    shooterTimer.reset();
                    shooterStarted = true;
                }

                if (pathStarted && !follower.isBusy()) {
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

         public void statePathUpdate( PathState newState) {
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

    motorPipeline = new Motor_PipeLine(this);
    Motor_PipeLine.resetMotors();
    flywheel = Motor_PipeLine.flywheel;
    intake = Motor_PipeLine.intake;

    buildPaths();
    follower.setPose(startPose);

    }

     public void start () {
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
    
