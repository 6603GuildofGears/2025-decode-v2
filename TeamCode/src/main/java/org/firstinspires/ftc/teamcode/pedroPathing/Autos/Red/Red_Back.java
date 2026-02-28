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
import org.firstinspires.ftc.teamcode.pedroPathing.TeleOp.ShooterLookup;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;

import static org.firstinspires.ftc.teamcode.pedroPathing.TeleOp.TurretConfig.*;

@Autonomous(name = "PEDRO - Red Back Auto", group = "Red")
public class Red_Back extends OpMode {
    private Follower follower;
    private Timer pathTimer, opmodeTimer;

    private DcMotorEx flywheel;
    private DcMotorEx intake;
    private DcMotorEx intakeMotor2;
    private DcMotorEx turret;
    private Motor_PipeLine motorPipeline;
    private Servo_Pipeline servoPipeline;
    private SpindexerController spindexerController;
    private boolean intakeRunning = false;
    private boolean shootSequenceStarted = false;
    private ElapsedTime ballWaitTimer = new ElapsedTime();
    private boolean ballWaitStarted = false;

    // Limelight-based shooter tuning
    private Limelight3A limelight;
    private double shootRpm = 4625;
    private double shootHood = 0.5;
    private double smoothedDistance = -1;
    private static final double SMOOTHING_ALPHA = 0.3;
    private static final int GOAL_TAG_ID = 24; // red goal

    // Turret auto-aim
    private static final double TURRET_SHOOT_DEG = 71.6;  // turret position to shoot
    private double turretTargetDeg = TURRET_SHOOT_DEG;     // active target
    private static final double TURRET_P_GAIN = 0.006;
    private static final double TURRET_MAX_POWER = 0.30;
    private static final double TURRET_LIMIT_DEG = 5.0;


        public enum PathState {
            TURRET_TO_SHOOT,
            SHOOT_PRELOAD,
            DRIVE_SHOOTPOSE_TO_INTAKE1,
            DRIVE_INTAKE1_TO_INTAKE1POSE2,
            DRIVE_INTAKE1POSE2_TO_INTAKE1POSE3,
            DRIVE_INTAKE1POSE3_TO_INTAKE2,
            DRIVE_INTAKE2_TO_INTAKE2POSE1,
            DRIVE_INTAKE2POSE1_TO_INTAKE2POSE2,
            DRIVE_INTAKE2POSE2_TO_INTAKE2POSE3,
            DRIVE_INTAKE2POSE3_TO_SHOOTPOSE,
            SHOOT_ALL,
            DRIVE_SHOOTPOSE_TO_ENDPOSE,
        }

              PathState pathState;


    // Poses — mirrored from Blue_Back (redX = 144 - blueX, same Y)
    private final Pose startPose = new Pose(87, 9, Math.toRadians(90));

    private final Pose shootPose = new Pose(87, 20, Math.toRadians(0));    // Shooting position

    private final Pose intake1 = new Pose(120, 36, Math.toRadians(0));       // Intake 1 position
    private final Pose intake1Pose2 = new Pose(123, 36, Math.toRadians(0));  // Intake 1 waypoint 2
    private final Pose intake1Pose3 = new Pose(126, 36, Math.toRadians(0));  // Intake 1 waypoint 3

    private final Pose intake2 = new Pose(134.5, 9.5, Math.toRadians(0));    // Intake 2 position
    private final Pose intake2Pose1 = new Pose(137.5, 9.5, Math.toRadians(0)); // Intake 2 waypoint 1
    private final Pose intake2Pose2 = new Pose(140.5, 9.5, Math.toRadians(0)); // Intake 2 waypoint 2
    private final Pose intake2Pose3 = new Pose(143, 9.5, Math.toRadians(0));   // Intake 2 waypoint 3

    private final Pose endPose = new Pose(132, 36, Math.toRadians(0));  // End position
   // Shooter hardware
   
    private ElapsedTime shooterTimer = new ElapsedTime();
    private boolean shooterStarted = false;
    private boolean pathStarted = false;


    private PathChain driveStartToIntake1;
    private PathChain driveShootPoseToIntake1;
    private PathChain driveIntake1ToIntake1Pose2;
    private PathChain driveIntake1Pose2ToIntake1Pose3;
    private PathChain driveIntake1Pose3ToIntake2;
    private PathChain driveIntake2ToIntake2Pose1;
    private PathChain driveIntake2Pose1ToIntake2Pose2;
    private PathChain driveIntake2Pose2ToIntake2Pose3;
    private PathChain driveIntake2Pose3ToShootPose;
    private PathChain driveShootPoseToEndPose;


 public void buildPaths() {

        driveStartToIntake1 = follower.pathBuilder()
            .addPath(new BezierLine(startPose, intake1))
            .setConstantHeadingInterpolation(intake1.getHeading())
            .build();

        driveShootPoseToIntake1 = follower.pathBuilder()
            .addPath(new BezierLine(startPose, intake1))
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

        driveIntake1Pose3ToIntake2 = follower.pathBuilder()
            .addPath(new BezierLine(intake1Pose3, intake2))
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
        double targetTicks = turretTargetDeg * TICKS_PER_DEG;
        double error = targetTicks - pos;
        double power = error * TURRET_P_GAIN;
        power = Math.max(-TURRET_MAX_POWER, Math.min(TURRET_MAX_POWER, power));
        if (posDeg > turretTargetDeg + TURRET_LIMIT_DEG && power > 0) {
            power = 0;
        } else if (posDeg < turretTargetDeg - TURRET_LIMIT_DEG && power < 0) {
            power = 0;
        }
        turret.setPower(power);
    }

    private boolean isTurretAtTarget(double toleranceDeg) {
        double posDeg = turret.getCurrentPosition() / TICKS_PER_DEG;
        return Math.abs(posDeg - turretTargetDeg) < toleranceDeg;
    }

    /**
     * Poll Limelight for distance to goal tag, use ShooterLookup to set RPM + hood.
     */
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
                        spindexerController.setShootRpm(shootRpm);
                        Servo_Pipeline.hood.setPosition(shootHood);
                    }
                }
            }
        } catch (Exception e) { /* keep last known values */ }
    }

    private double getTickSpeed(double rpm) { return rpm * 28.0 / 60.0; }

     public void statePathUpdate() {
        updateTurret();
        updateShooterFromLimelight();

        switch (pathState) {
            case TURRET_TO_SHOOT:
                // Spin up flywheel, wait for turret to reach shoot position
                if (!shooterStarted) {
                    turretTargetDeg = TURRET_SHOOT_DEG;
                    Servo_Pipeline.spindexerAxon.setTargetRotation(SpindexerController.P1);
                    flywheel.setVelocity(getTickSpeed(shootRpm));
                    shooterTimer.reset();
                    shooterStarted = true;
                }
                if (isTurretAtTarget(2.0) || shooterTimer.seconds() >= 2.0) {
                    pathState = PathState.SHOOT_PRELOAD;
                    shooterTimer.reset();
                    shooterStarted = false;
                }
                break;

            case SHOOT_PRELOAD:
                if (!shootSequenceStarted) {
                    spindexerController.prefillAllSlots();
                    spindexerController.updateShoot(true, false, flywheel);
                    intake.setPower(-0.5);
                    intakeMotor2.setPower(-0.5);
                    shootSequenceStarted = true;
                } else {
                    spindexerController.updateShoot(false, false, flywheel);
                }
                if (!spindexerController.isShooting() && shootSequenceStarted) {
                    intake.setPower(0);
                    intakeMotor2.setPower(0);
                    flywheel.setVelocity(0);
                    shootSequenceStarted = false;
                    spindexerController.resetShootState();
                    spindexerController.clearAllSlots();
                    Servo_Pipeline.flicker.setPosition(0);
                    pathState = PathState.DRIVE_SHOOTPOSE_TO_INTAKE1;
                }
                break;

            case DRIVE_SHOOTPOSE_TO_INTAKE1:
                if (!pathStarted) {
                    follower.followPath(driveShootPoseToIntake1, 0.425, true);
                    pathStarted = true;

                    intake.setPower(-0.275);
                    intakeMotor2.setPower(-0.275);
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

                    intake.setPower(-0.275);
                    intakeMotor2.setPower(-0.275);
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

                    intake.setPower(-0.275);
                    intakeMotor2.setPower(-0.275);
                    intakeRunning = true;
                }

                if (pathStarted && !follower.isBusy()) {
                    if (!ballWaitStarted) {
                        ballWaitTimer.reset();
                        ballWaitStarted = true;
                    }
                    if (Sensor.isBallPresent() || ballWaitTimer.seconds() >= 1.0) {
                        pathState = PathState.DRIVE_INTAKE1POSE3_TO_INTAKE2;
                        pathStarted = false;
                        ballWaitStarted = false;
                    }
                }
                break;

            case DRIVE_INTAKE1POSE3_TO_INTAKE2:
                if (!pathStarted) {
                    follower.followPath(driveIntake1Pose3ToIntake2, true);
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

                    intake.setPower(-0.275);
                    intakeMotor2.setPower(-0.275);
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

                    intake.setPower(-0.275);
                    intakeMotor2.setPower(-0.275);
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

                    intake.setPower(-0.275);
                    intakeMotor2.setPower(-0.275);
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
                    flywheel.setVelocity(getTickSpeed(shootRpm));
                    pathStarted = true;
                }

                if (follower.getCurrentTValue() >= 0.5) {
                    intake.setPower(0);
                    intakeMotor2.setPower(0);
                    intakeRunning = false;
                }

                if (pathStarted && !follower.isBusy()) {
                    pathState = PathState.SHOOT_ALL;
                    pathStarted = false;
                    shooterTimer.reset();
                }
                break;

            case SHOOT_ALL:
                if (!shootSequenceStarted) {
                    spindexerController.prefillAllSlots();
                    spindexerController.updateShoot(true, false, flywheel);
                    intake.setPower(-0.5);
                    intakeMotor2.setPower(-0.5);
                    shootSequenceStarted = true;
                } else {
                    spindexerController.updateShoot(false, false, flywheel);
                }
                if (!spindexerController.isShooting() && shootSequenceStarted) {
                    intake.setPower(0);
                    intakeMotor2.setPower(0);
                    flywheel.setVelocity(0);
                    shootSequenceStarted = false;
                    spindexerController.resetShootState();
                    spindexerController.clearAllSlots();
                    Servo_Pipeline.flicker.setPosition(0);
                    pathState = PathState.DRIVE_SHOOTPOSE_TO_ENDPOSE;
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
                    turret.setPower(0);
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
    
    pathState = PathState.TURRET_TO_SHOOT;
    pathTimer = new Timer();
    opmodeTimer = new Timer();
    opmodeTimer.resetTimer();
    follower = Constants.createFollower(hardwareMap);

    motorPipeline = new Motor_PipeLine(this);
    Motor_PipeLine.resetMotors();
    flywheel = Motor_PipeLine.flywheel;
    intake = Motor_PipeLine.intake;
    intakeMotor2 = Motor_PipeLine.intake2;
    turret = Motor_PipeLine.turret;

    Sensor.initSensors(this);
    servoPipeline = new Servo_Pipeline(this);
    spindexerController = new SpindexerController();
    spindexerController.setFlickerPositions(0, 0.375);
    spindexerController.setShootRpm(shootRpm);
    spindexerController.prefillAllSlots();
    Servo_Pipeline.flicker.setPosition(0);
    Servo_Pipeline.spindexerAxon.setTargetRotation(SpindexerController.P1);

    limelight = hardwareMap.get(Limelight3A.class, "limelight");

    buildPaths();
    follower.setPose(startPose);

    }

     public void start () {
    opmodeTimer.resetTimer();
    pathState = PathState.TURRET_TO_SHOOT;
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
        telemetry.addData("Shooter rpm", flywheel.getVelocity() * 60.0 / 28.0);
        telemetry.addData("Target rpm", shootRpm);
        telemetry.update();
    
    }



 } 
    
