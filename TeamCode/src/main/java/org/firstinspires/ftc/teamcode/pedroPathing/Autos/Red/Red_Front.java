package org.firstinspires.ftc.teamcode.pedroPathing.Autos.Red;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.pedropathing.util.Timer;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.*;
import com.pedropathing.paths.*;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.pedroPathing.other.Constants;
import org.firstinspires.ftc.teamcode.pedroPathing.Pipelines.Motor_PipeLine;
import org.firstinspires.ftc.teamcode.pedroPathing.Pipelines.Sensor;
import org.firstinspires.ftc.teamcode.pedroPathing.Pipelines.Servo_Pipeline;
import org.firstinspires.ftc.teamcode.pedroPathing.Pipelines.SpindexerController;

import static org.firstinspires.ftc.teamcode.pedroPathing.TeleOp.TurretConfig.*;

@Autonomous(name = "PEDRO - Red Front Auto", group = "Red")
public class Red_Front extends OpMode {
    private Follower follower;
    private Timer pathTimer, opmodeTimer;

    private DcMotorEx flywheel;
    private DcMotorEx intake;
    private DcMotorEx turret;
    private Motor_PipeLine motorPipeline;
    private Servo_Pipeline servoPipeline;
    private SpindexerController spindexerController;
    private boolean intakeRunning = false;
    private boolean shootSequenceStarted = false;
    private ElapsedTime ballWaitTimer = new ElapsedTime();
    private boolean ballWaitStarted = false;

    // Turret auto-aim
    private static final double TURRET_TARGET_DEG = 85.0;
    private static final double TURRET_P_GAIN = 0.006;
    private static final double TURRET_MAX_POWER = 0.30;
    private static final double TURRET_LIMIT_DEG = 5.0;

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

    // Poses from the provided coordinates
    private final Pose startPose = new Pose(123, 123, Math.toRadians(37));
    private final Pose shootPose = new Pose(85, 85, Math.toRadians(0));
    private final Pose intake1 = new Pose(113, 84, Math.toRadians(0));       // Intake 1 position
    private final Pose intake1Pose2 = new Pose(117, 84, Math.toRadians(0));   // Intake 1 waypoint 2
    private final Pose intake1Pose3 = new Pose(121, 84, Math.toRadians(0));   // Intake 1 waypoint 3
    private final Pose intake2 = new Pose(113, 60, Math.toRadians(0));       // Intake 2 position
    private final Pose intake2Pose1 = new Pose(117, 60, Math.toRadians(0));   // Intake 2 waypoint 1
    private final Pose intake2Pose2 = new Pose(121, 60, Math.toRadians(0));   // Intake 2 waypoint 2
    private final Pose intake2Pose3 = new Pose(125, 60, Math.toRadians(0));   // Intake 2 waypoint 3
    private final Pose endPose = new Pose(125, 70, Math.toRadians(0));       // End/park position

    private ElapsedTime shooterTimer = new ElapsedTime();
    private boolean shooterStarted = false;
    private boolean pathStarted = false;

    private PathChain driveStartPoseToShootPose;
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

    private void updateTurret() {
        int pos = turret.getCurrentPosition();
        double posDeg = pos / TICKS_PER_DEG;
        double targetTicks = TURRET_TARGET_DEG * TICKS_PER_DEG;
        double error = targetTicks - pos;
        double power = error * TURRET_P_GAIN;
        power = Math.max(-TURRET_MAX_POWER, Math.min(TURRET_MAX_POWER, power));
        if (posDeg > TURRET_TARGET_DEG + TURRET_LIMIT_DEG && power > 0) {
            power = 0;
        } else if (posDeg < TURRET_TARGET_DEG - TURRET_LIMIT_DEG && power < 0) {
            power = 0;
        }
        turret.setPower(power);
    }

    public void statePathUpdate() {
        updateTurret();

        switch (pathState) {
            case DRIVE_STARTPOSE_TO_SHOOTPOSE:
                if (!pathStarted) {
                    follower.followPath(driveStartPoseToShootPose, true);
                    Servo_Pipeline.spindexer.setPosition(SpindexerController.P1);
                    flywheel.setVelocity(3000.0 * 28.0 / 60.0);
                    pathStarted = true;
                }
             
                if (pathStarted && !follower.isBusy()) {
                    pathState = PathState.SHOOT_PRELOAD;
                    pathStarted = false;
                    shooterTimer.reset();
                }
                break;

            case SHOOT_PRELOAD:
                if (!shootSequenceStarted) {
                    spindexerController.prefillAllSlots();
                    spindexerController.updateShoot(true, false, flywheel);
                    intake.setPower(-0.5);
                    shootSequenceStarted = true;
                } else {
                    spindexerController.updateShoot(false, false, flywheel);
                }

                if (!spindexerController.isShooting() && shootSequenceStarted) {
                    intake.setPower(0);
                    flywheel.setVelocity(0);
                    shootSequenceStarted = false;
                    spindexerController.resetShootState();
                    spindexerController.clearAllSlots();
                    Servo_Pipeline.flicker1.setPosition(0.1);
                    Servo_Pipeline.flicker2.setPosition(0.0875);
                    pathState = PathState.DRIVE_SHOOTPOSE_TO_INTAKE1;
                }

                telemetry.addLine("Preload Shot");
                break;

            case DRIVE_SHOOTPOSE_TO_INTAKE1:
                if (!pathStarted) {
                    follower.followPath(driveShootPoseToIntake1, 0.425, true);
                    pathStarted = true;

                    intake.setPower(-0.275); // Start intake while driving to first intake position
                    intakeRunning = true;
                }

                if (pathStarted && !follower.isBusy()) {
                    if (!ballWaitStarted) {
                        ballWaitTimer.reset();
                        ballWaitStarted = true;
                    }
                    if (Sensor.isBallPresent() || ballWaitTimer.seconds() >= 1.0) {
                        pathState = PathState.DRIVE_INTAKE1_TO_INTAKE1POSE2;
                        pathStarted = false;
                        ballWaitStarted = false;
                    }
                }
                break;

            case DRIVE_INTAKE1_TO_INTAKE1POSE2:
                if (!pathStarted) {
                    follower.followPath(driveIntake1ToIntake1Pose2, 0.425, true);
                    pathStarted = true;

                    intake.setPower(-0.275); // Keep intake running while driving to second intake position
                    intakeRunning = true;
                }

                if (pathStarted && !follower.isBusy()) {
                    if (!ballWaitStarted) {
                        ballWaitTimer.reset();
                        ballWaitStarted = true;
                    }
                    if (Sensor.isBallPresent() || ballWaitTimer.seconds() >= 1.0) {
                        pathState = PathState.DRIVE_INTAKE1POSE2_TO_INTAKE1POSE3;
                        pathStarted = false;
                        ballWaitStarted = false;
                    }
                }
                break;

            case DRIVE_INTAKE1POSE2_TO_INTAKE1POSE3:
                if (!pathStarted) {
                    follower.followPath(driveIntake1Pose2ToIntake1Pose3, 0.425, true);
                    pathStarted = true;

                    intake.setPower(-0.275); // Keep intake running while driving to third intake position
                    intakeRunning = true;
                }

                if (pathStarted && !follower.isBusy()) {
                    if (!ballWaitStarted) {
                        ballWaitTimer.reset();
                        ballWaitStarted = true;
                    }
                    if (Sensor.isBallPresent() || ballWaitTimer.seconds() >= 1.0) {
                        pathState = PathState.DRIVE_INTAKE1POSE3_TO_SHOOTPOSE;
                        pathStarted = false;
                        ballWaitStarted = false;
                    }
                }
                break;

            case DRIVE_INTAKE1POSE3_TO_SHOOTPOSE:
                if (!pathStarted) {
                    follower.followPath(driveIntake1Pose3ToShootPose, true);
                    flywheel.setVelocity(3000.0 * 28.0 / 60.0);
                    pathStarted = true;
                }

                if (follower.getCurrentTValue() >= 0.5) {
                    intake.setPower(0); // Stop intake halfway through drive back to shoot position
                    intakeRunning = false;
                }

                if (pathStarted && !follower.isBusy()) {
                    pathState = PathState.SHOOT_INTAKE1;
                    pathStarted = false;
                    shooterTimer.reset();
                }
                break;

            case SHOOT_INTAKE1:
                if (!shootSequenceStarted) {
                    spindexerController.updateShoot(true, false, flywheel);
                    intake.setPower(-0.5);
                    shootSequenceStarted = true;
                } else {
                    spindexerController.updateShoot(false, false, flywheel);
                }

                if (!spindexerController.isShooting() && shootSequenceStarted) {
                    intake.setPower(0);
                    flywheel.setVelocity(0);
                    shootSequenceStarted = false;
                    spindexerController.resetShootState();
                    spindexerController.clearAllSlots();
                    Servo_Pipeline.flicker1.setPosition(0.1);
                    Servo_Pipeline.flicker2.setPosition(0.0875);
                    pathState = PathState.DRIVE_SHOOTPOSE_TO_INTAKE2;
                }

                telemetry.addLine("Sample 1 Shot");
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
                    follower.followPath(driveIntake2ToIntake2Pose1, 0.425, true);
                    pathStarted = true;

                    intake.setPower(-0.275); // Start intake while driving to first intake position
                    intakeRunning = true;
                }

                if (pathStarted && !follower.isBusy()) {
                    if (!ballWaitStarted) {
                        ballWaitTimer.reset();
                        ballWaitStarted = true;
                    }
                    if (Sensor.isBallPresent() || ballWaitTimer.seconds() >= 1.0) {
                        pathState = PathState.DRIVE_INTAKE2POSE1_TO_INTAKE2POSE2;
                        pathStarted = false;
                        ballWaitStarted = false;
                    }
                }
                break;

            case DRIVE_INTAKE2POSE1_TO_INTAKE2POSE2:
                if (!pathStarted) {
                    follower.followPath(driveIntake2Pose1ToIntake2Pose2, 0.425, true);
                    pathStarted = true;

                    intake.setPower(-0.275); // Keep intake running while driving to second intake position
                    intakeRunning = true;
                }

                if (pathStarted && !follower.isBusy()) {
                    if (!ballWaitStarted) {
                        ballWaitTimer.reset();
                        ballWaitStarted = true;
                    }
                    if (Sensor.isBallPresent() || ballWaitTimer.seconds() >= 1.0) {
                        pathState = PathState.DRIVE_INTAKE2POSE2_TO_INTAKE2POSE3;
                        pathStarted = false;
                        ballWaitStarted = false;
                    }
                }
                break;

            case DRIVE_INTAKE2POSE2_TO_INTAKE2POSE3:
                if (!pathStarted) {
                    follower.followPath(driveIntake2Pose2ToIntake2Pose3, 0.425, true);
                    pathStarted = true;

                    intake.setPower(-0.275); // Keep intake running while driving to third intake position
                    intakeRunning = true;
                }

                if (pathStarted && !follower.isBusy()) {
                    if (!ballWaitStarted) {
                        ballWaitTimer.reset();
                        ballWaitStarted = true;
                    }
                    if (Sensor.isBallPresent() || ballWaitTimer.seconds() >= 1.0) {
                        pathState = PathState.DRIVE_INTAKE2POSE3_TO_SHOOTPOSE;
                        pathStarted = false;
                        ballWaitStarted = false;
                    }
                }
                break;

            case DRIVE_INTAKE2POSE3_TO_SHOOTPOSE:
                if (!pathStarted) {
                    follower.followPath(driveIntake2Pose3ToShootPose, true);
                    flywheel.setVelocity(3000.0 * 28.0 / 60.0);
                    pathStarted = true;
                }
                

                if (follower.getCurrentTValue() >= 0.5) {
                    intake.setPower(0); // Stop intake halfway through drive back to shoot position
                    intakeRunning = false;
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
                if (!shootSequenceStarted) {
                    spindexerController.updateShoot(true, false, flywheel);
                    intake.setPower(-0.5);
                    shootSequenceStarted = true;
                } else {
                    spindexerController.updateShoot(false, false, flywheel);
                }

                if (!spindexerController.isShooting() && shootSequenceStarted) {
                    intake.setPower(0);
                    flywheel.setVelocity(0);
                    shootSequenceStarted = false;
                    spindexerController.resetShootState();
                    spindexerController.clearAllSlots();
                    Servo_Pipeline.flicker1.setPosition(0.1);
                    Servo_Pipeline.flicker2.setPosition(0.0875);
                    pathState = PathState.DRIVE_SHOOTPOSE_TO_ENDPOSE;
                }

                telemetry.addLine("Sample 2 Shot");
                break;

            case DRIVE_SHOOTPOSE_TO_ENDPOSE:
                if (!pathStarted) {
                    follower.followPath(driveShootPoseToEndPose, true);
                    pathStarted = true;
                }

                if (pathStarted && !follower.isBusy()) {
                    telemetry.addLine("Auto Complete");
                    pathStarted = false;
                    turret.setPower(0);
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

        motorPipeline = new Motor_PipeLine(this);
        Motor_PipeLine.resetMotors();
        flywheel = Motor_PipeLine.flywheel;
        intake = Motor_PipeLine.intake;
        turret = Motor_PipeLine.turret;

        Sensor.initSensors(this);
        servoPipeline = new Servo_Pipeline(this);
        spindexerController = new SpindexerController();
        spindexerController.setFlickerPositions(0.1, 0.0875, 0.5, 0.5);
        spindexerController.setShootRpm(3000);
        spindexerController.prefillAllSlots();
        Servo_Pipeline.flicker1.setPosition(0.1);
        Servo_Pipeline.flicker2.setPosition(0.0875);
        Servo_Pipeline.spindexer.setPosition(SpindexerController.P1);

        buildPaths();
        follower.setPose(startPose);
    }

    public void start() {
        opmodeTimer.resetTimer();
        pathState = PathState.DRIVE_STARTPOSE_TO_SHOOTPOSE;
        spindexerController.goToSlot(0);
    }

    @Override
    public void loop() {
        follower.update();

        statePathUpdate();
        spindexerController.updateIntake(intakeRunning);

        telemetry.addData("Path state", pathState.toString());
        telemetry.addData("X", follower.getPose().getX());
        telemetry.addData("Y", follower.getPose().getY());
        telemetry.addData("Heading", Math.toDegrees(follower.getPose().getHeading()));
        telemetry.addData("Path time (s): ", pathTimer.getElapsedTimeSeconds());
        telemetry.update();
    }
}
