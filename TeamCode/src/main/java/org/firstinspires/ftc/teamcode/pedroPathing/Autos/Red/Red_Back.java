package org.firstinspires.ftc.teamcode.pedroPathing.Autos.Red;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.pedropathing.util.Timer;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.*;
import com.pedropathing.paths.*;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.pedroPathing.other.Constants;
import org.firstinspires.ftc.teamcode.pedroPathing.Pipelines.SpindexerController;
import com.bylazar.configurables.PanelsConfigurables;
import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;

import static org.firstinspires.ftc.teamcode.pedroPathing.Pipelines.Servo_Pipeline.*;
import static org.firstinspires.ftc.teamcode.pedroPathing.Pipelines.Sensor.*;
import static org.firstinspires.ftc.teamcode.pedroPathing.TeleOp.TurretConfig.*;

@Configurable
@Autonomous(name = "PEDRO - Red Back Auto", group = "Red")
public class Red_Back extends OpMode {
    private Follower follower;
    private Timer pathTimer, opmodeTimer;

    // Hardware (non-drive motors)
    private DcMotorEx turret;
    private DcMotorEx flywheel;
    private DcMotorEx intakeMotor;

    // Spindexer controller
    private SpindexerController sdx;

    // Panels telemetry
    private TelemetryManager telemetryM;

    // ========== Turret auto-aim ==========
    public static double TURRET_TARGET_DEG = 35.0;   // turret angle for red back
    public static double TURRET_P_GAIN = 0.006;
    public static double TURRET_MAX_POWER = 0.30;
    public static double TURRET_LIMIT_DEG = 5.0;

    // ========== Shooter tuning ==========
    public static double AUTO_SHOOT_RPM = 4375;
    public static double AUTO_HOOD_POS  = 0.5;

    public enum PathState {
        DRIVE_STARTPOSE_TO_SHOOTPOSE,
        SHOOT_PRELOAD,
        DRIVE_SHOOT_PRELOAD_TO_INTAKEPOSE,
        DRIVE_INTAKEPOSE_TO_INTAKE1,
        DRIVE_INTAKE1_TO_SHOOTPOSE,
        SHOOT_INTAKE1,
        DRIVE_SHOOTPOSE_TO_INTAKEPOSE2,
        DRIVE_INTAKEPOSE2_TO_INTAKE2,
        DRIVE_INTAKE2_TO_SHOOTPOSE,
        SHOOT_INTAKE2,
        DRIVE_SHOOTPOSE_TO_ENDPOSE,
    }

    PathState pathState;

    private final Pose startPose = new Pose(84, 9, Math.toRadians(0));
    private final Pose shootPose = new Pose(84, 17, Math.toRadians(64));
    private final Pose intakePose = new Pose(103, 36, Math.toRadians(0));
    private final Pose intake1 = new Pose(120, 36, Math.toRadians(0));
    private final Pose intakePose2 = new Pose(135, 9.5, Math.toRadians(0));
    private final Pose intake2 = new Pose(135, 9.5, Math.toRadians(0));
    private final Pose endPose = new Pose(120, 36, Math.toRadians(0));

    private ElapsedTime shooterTimer = new ElapsedTime();
    private boolean shooterStarted = false;
    private boolean pathStarted = false;
    private boolean shootTriggered = false;
    private boolean wasShootingLast = false;
    private boolean currentlyShooting = false;
    private int shootAttempts = 0;

    private PathChain driveStartPoseShootPose;
    private PathChain driveShootPreloadToIntakePose;
    private PathChain driveIntakePoseToIntake1;
    private PathChain driveIntake1ToShootPose;
    private PathChain driveShootPoseToIntakePose2;
    private PathChain driveIntakePose2ToIntake2;
    private PathChain driveIntake2ToShootPose;
    private PathChain driveShootPoseToEndPose;

    public void buildPaths() {
        driveStartPoseShootPose = follower.pathBuilder()
                .addPath(new BezierLine(startPose, shootPose))
                .setConstantHeadingInterpolation(shootPose.getHeading())
                .build();
        driveShootPreloadToIntakePose = follower.pathBuilder()
                .addPath(new BezierLine(shootPose, intakePose))
                .setConstantHeadingInterpolation(intakePose.getHeading())
                .build();
        driveIntakePoseToIntake1 = follower.pathBuilder()
                .addPath(new BezierLine(intakePose, intake1))
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

    // ======================================================================
    //  TURRET P-CONTROL
    // ======================================================================
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

    // ======================================================================
    //  STATE MACHINE
    // ======================================================================
    public void statePathUpdate() {
        updateTurret();

        boolean wantIntakeMotor = false;

        switch (pathState) {

            case DRIVE_STARTPOSE_TO_SHOOTPOSE:
                hood.setPosition(AUTO_HOOD_POS);


                if (!pathStarted) {
                    follower.followPath(driveStartPoseShootPose, true);
                    spindexer.setPosition(0.02);
                    hood.setPosition(AUTO_HOOD_POS);
                    pathStarted = true;
                }
                if (!follower.isBusy()) {
                    pathState = PathState.SHOOT_PRELOAD;
                    pathStarted = false;
                    shootTriggered = false;
                    wasShootingLast = false;
                    shootAttempts = 0;
                    shooterTimer.reset();
                    sdx.resetShootState();
                }
                break;

            case SHOOT_PRELOAD:
                if (!shootTriggered) {
                    sdx.setShootRpm(AUTO_SHOOT_RPM);
                    sdx.updateShoot(true, false, flywheel);
                    shootTriggered = true;
                    wasShootingLast = true;
                } else {
                    sdx.updateShoot(false, false, flywheel);
                }

                currentlyShooting = sdx.isShooting();
                if (wasShootingLast && !currentlyShooting) {
                    pathState = PathState.DRIVE_SHOOT_PRELOAD_TO_INTAKEPOSE;
                    pathStarted = false;
                    shooterStarted = false;
                    break;
                }
                wasShootingLast = currentlyShooting || wasShootingLast;

                if (shooterTimer.seconds() >= 10) {
                    shootAttempts++;
                    if (shootAttempts >= 3) {
                        // 3 failures — skip intake to avoid penalties
                        flywheel.setPower(0);
                        pathState = PathState.DRIVE_SHOOTPOSE_TO_ENDPOSE;
                        pathStarted = false;
                        shooterStarted = false;
                    } else {
                        // Retry shoot — slot tracking from RPM drops preserved
                        sdx.resetShootState();
                        shootTriggered = false;
                        wasShootingLast = false;
                        shooterTimer.reset();
                    }
                }
                break;

            case DRIVE_SHOOT_PRELOAD_TO_INTAKEPOSE:
                if (!pathStarted) {
                    follower.followPath(driveShootPreloadToIntakePose, 1, true);
                    flywheel.setPower(0);
                    pathStarted = true;
                }
                wantIntakeMotor = true;
                if (pathStarted && !follower.isBusy()) {
                    pathState = PathState.DRIVE_INTAKEPOSE_TO_INTAKE1;
                    pathStarted = false;
                }
                break;

            case DRIVE_INTAKEPOSE_TO_INTAKE1:
                if (!pathStarted) {
                    follower.followPath(driveIntakePoseToIntake1, 0.375, true);
                    pathStarted = true;
                }
                wantIntakeMotor = true;
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
                wantIntakeMotor = true;

                if (!follower.isBusy()) {
                    pathState = PathState.SHOOT_INTAKE1;
                    pathStarted = false;
                    shootTriggered = false;
                    wasShootingLast = false;
                    shootAttempts = 0;
                    shooterTimer.reset();
                    sdx.prefillAllSlots();
                    sdx.resetShootState();
                    hood.setPosition(AUTO_HOOD_POS);
                }
                break;

            case SHOOT_INTAKE1:
                if (!shootTriggered) {
                    sdx.setShootRpm(AUTO_SHOOT_RPM);
                    sdx.updateShoot(true, false, flywheel);
                    shootTriggered = true;
                    wasShootingLast = true;
                } else {
                    sdx.updateShoot(false, false, flywheel);
                }

                currentlyShooting = sdx.isShooting();
                if (wasShootingLast && !currentlyShooting) {
                    pathState = PathState.DRIVE_SHOOTPOSE_TO_INTAKEPOSE2;
                    pathStarted = false;
                    shooterStarted = false;
                    break;
                }
                wasShootingLast = currentlyShooting || wasShootingLast;

                if (shooterTimer.seconds() >= 10) {
                    shootAttempts++;
                    if (shootAttempts >= 3) {
                        // 3 failures — skip intake to avoid penalties
                        flywheel.setPower(0);
                        pathState = PathState.DRIVE_SHOOTPOSE_TO_ENDPOSE;
                        pathStarted = false;
                        shooterStarted = false;
                    } else {
                        // Retry shoot — slot tracking from RPM drops preserved
                        sdx.resetShootState();
                        shootTriggered = false;
                        wasShootingLast = false;
                        shooterTimer.reset();
                    }
                }
                break;

            case DRIVE_SHOOTPOSE_TO_INTAKEPOSE2:
                if (!pathStarted) {
                    follower.followPath(driveShootPoseToIntakePose2, 0.375, true);
                    flywheel.setPower(0);
                    pathStarted = true;
                }
                wantIntakeMotor = true;
                if (pathStarted && !follower.isBusy()) {
                    pathState = PathState.DRIVE_INTAKEPOSE2_TO_INTAKE2;
                    pathStarted = false;
                }
                break;

            case DRIVE_INTAKEPOSE2_TO_INTAKE2:
                if (!pathStarted) {
                    follower.followPath(driveIntakePose2ToIntake2, 0.25, true);
                    pathStarted = true;
                }
                wantIntakeMotor = true;
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
                wantIntakeMotor = true;

                if (!follower.isBusy()) {
                    pathState = PathState.SHOOT_INTAKE2;
                    pathStarted = false;
                    shootTriggered = false;
                    wasShootingLast = false;
                    shootAttempts = 0;
                    shooterTimer.reset();
                    sdx.prefillAllSlots();
                    sdx.resetShootState();
                    hood.setPosition(AUTO_HOOD_POS);
                }
                break;

            case SHOOT_INTAKE2:
                if (!shootTriggered) {
                    sdx.setShootRpm(AUTO_SHOOT_RPM);
                    sdx.updateShoot(true, false, flywheel);
                    shootTriggered = true;
                    wasShootingLast = true;
                } else {
                    sdx.updateShoot(false, false, flywheel);
                }

                currentlyShooting = sdx.isShooting();
                if (wasShootingLast && !currentlyShooting) {
                    pathState = PathState.DRIVE_SHOOTPOSE_TO_ENDPOSE;
                    pathStarted = false;
                    shooterStarted = false;
                    break;
                }
                wasShootingLast = currentlyShooting || wasShootingLast;

                if (shooterTimer.seconds() >= 10) {
                    shootAttempts++;
                    if (shootAttempts >= 3) {
                        pathState = PathState.DRIVE_SHOOTPOSE_TO_ENDPOSE;
                        pathStarted = false;
                        shooterStarted = false;
                    } else {
                        // Retry shoot — slot tracking from RPM drops preserved
                        sdx.resetShootState();
                        shootTriggered = false;
                        wasShootingLast = false;
                        shooterTimer.reset();
                    }
                }
                break;

            case DRIVE_SHOOTPOSE_TO_ENDPOSE:
                if (!pathStarted) {
                    follower.followPath(driveShootPoseToEndPose, true);
                    flywheel.setPower(0);
                    pathStarted = true;
                }
                if (pathStarted && !follower.isBusy()) {
                    turret.setPower(0);
                    telemetry.addLine("Auto Complete");
                    pathStarted = false;
                    requestOpModeStop();
                }
                break;

            default:
                telemetry.addLine("No valid path state");
                break;
        }

        // Run intake motor when requested by state machine
        boolean sdxWantsMotor = sdx.updateIntake(wantIntakeMotor);
        if (wantIntakeMotor && sdxWantsMotor) {
            intakeMotor.setPower(1.0);
        } else if (sdx.isShooting()) {
            intakeMotor.setPower(0.25);
        } else {
            intakeMotor.setPower(0);
        }
    }

    @Override
    public void init() {
        pathState = PathState.DRIVE_STARTPOSE_TO_SHOOTPOSE;
        pathTimer = new Timer();
        opmodeTimer = new Timer();
        opmodeTimer.resetTimer();

        follower = Constants.createFollower(hardwareMap);

        intServos(this);
        initSensors(this);

        turret = hardwareMap.get(DcMotorEx.class, "turret");
        turret.setDirection(DcMotor.Direction.FORWARD);
        turret.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        turret.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        turret.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        flywheel = hardwareMap.get(DcMotorEx.class, "flywheel");
        flywheel.setDirection(DcMotor.Direction.REVERSE);
        flywheel.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        intakeMotor = hardwareMap.get(DcMotorEx.class, "intake");
        intakeMotor.setDirection(DcMotor.Direction.FORWARD);
        intakeMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        sdx = new SpindexerController();
        sdx.setFlickerPositions(0.1, 0.0875, 0.5, 0.5);
        sdx.setShootRpm(AUTO_SHOOT_RPM);
        sdx.prefillAllSlots();
        flicker1.setPosition(0.1);
        flicker2.setPosition(0.0875);
        spindexer.setPosition(SpindexerController.P1);


        telemetryM = PanelsTelemetry.INSTANCE.getTelemetry();

        buildPaths();
        follower.setPose(startPose);

        telemetry.addData("Status", "Red Back Auto — Ready");
        telemetry.addData("Turret Target", TURRET_TARGET_DEG + "°");
        telemetry.addData("Shoot RPM", AUTO_SHOOT_RPM);
        telemetry.update();
    }

    @Override
    public void init_loop() {
        if (gamepad1.a && gamepad1.b && gamepad1.x && gamepad1.y) {
            sdx.clearAllSlots();
        }
        updateSensors();
        sdx.addTelemetry(telemetry);
        telemetry.update();
    }

    @Override
    public void start() {
        opmodeTimer.resetTimer();
        pathState = PathState.DRIVE_STARTPOSE_TO_SHOOTPOSE;
    }

    @Override
    public void loop() {
        PanelsConfigurables.INSTANCE.refreshClass(this);
        follower.update();
        statePathUpdate();

        telemetry.addData("Path state", pathState.toString());
        telemetry.addData("X", follower.getPose().getX());
        telemetry.addData("Y", follower.getPose().getY());
        telemetry.addData("Heading", Math.toDegrees(follower.getPose().getHeading()));
        telemetry.addData("Turret pos", turret.getCurrentPosition() / TICKS_PER_DEG + "°");
        sdx.addTelemetry(telemetry);

        telemetryM.debug("State: " + pathState.toString());
        telemetryM.debug("RPM: " + String.format("%.0f", AUTO_SHOOT_RPM) + " | Flywheel: " + String.format("%.0f", flywheel.getVelocity()));
        telemetryM.debug("Turret: " + String.format("%.1f°", turret.getCurrentPosition() / TICKS_PER_DEG));
        telemetryM.debug("Pos: " + String.format("%.1f, %.1f", follower.getPose().getX(), follower.getPose().getY()));
        telemetry.update();
        telemetryM.update();
    }
}
