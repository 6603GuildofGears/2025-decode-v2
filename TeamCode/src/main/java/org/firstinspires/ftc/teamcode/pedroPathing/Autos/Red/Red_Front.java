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
import org.firstinspires.ftc.teamcode.pedroPathing.TeleOp.ShooterLookup;

import static org.firstinspires.ftc.teamcode.pedroPathing.Pipelines.Servo_Pipeline.*;
import static org.firstinspires.ftc.teamcode.pedroPathing.Pipelines.Sensor.*;
import static org.firstinspires.ftc.teamcode.pedroPathing.TeleOp.TurretConfig.*;

@Autonomous(name = "PEDRO - Red Front Auto", group = "Red")
public class Red_Front extends OpMode {
    private Follower follower;
    private Timer pathTimer, opmodeTimer;

    // Hardware (non-drive motors — drive is handled by Pedro Follower)
    private DcMotorEx turret;
    private DcMotorEx flywheel;
    private DcMotorEx intakeMotor;

    // Spindexer controller
    private SpindexerController sdx;

    // ========== Turret auto-aim ==========
    private static final double TURRET_TARGET_DEG = 5;   // turret stays at 0° for red side
    private static final double TURRET_P_GAIN = 0.006;     // P-control gain (ticks)
    private static final double TURRET_MAX_POWER = 0.30;   // max turret power in auto
    private static final double TURRET_LIMIT_DEG  = 5.0;    // soft limit — ±5° from target

    // ========== Shooter tuning — uses ShooterLookup ==========
    private static final double AUTO_SHOOT_DISTANCE = 50.0; // inches from goal — tune this!
    private double autoShootRpm;   // set from lookup in init
    private double autoHoodPos;    // set from lookup in init

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

    // Poses — red side coordinates
    private final Pose startPose = new Pose(123, 123, Math.toRadians(37));
    private final Pose shootPose = new Pose(85, 85, Math.toRadians(0));
    private final Pose intake1 = new Pose(121, 80, Math.toRadians(0));       // Intake 1 position
    private final Pose intakePose2 = new Pose(102, 64, Math.toRadians(0));   // Drive to intake 2
    private final Pose intake2 = new Pose(121, 64, Math.toRadians(0));       // Intake 2 position
    private final Pose endPose = new Pose(121, 40, Math.toRadians(0));       // Park position

    private ElapsedTime shooterTimer = new ElapsedTime();
    private boolean shooterStarted = false;
    private boolean pathStarted = false;
    private boolean shootTriggered = false;
    private boolean wasShootingLast = false;
    private int shootAttempts = 0;

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

    // ======================================================================
    //  TURRET P-CONTROL — hold at target angle
    // ======================================================================
    private void updateTurret() {
        int pos = turret.getCurrentPosition();
        double posDeg = pos / TICKS_PER_DEG;
        double targetTicks = TURRET_TARGET_DEG * TICKS_PER_DEG;
        double error = targetTicks - pos;
        double power = error * TURRET_P_GAIN;
        power = Math.max(-TURRET_MAX_POWER, Math.min(TURRET_MAX_POWER, power));

        // Soft limits: if beyond target ± 5°, only allow power back toward target
        if (posDeg > TURRET_TARGET_DEG + TURRET_LIMIT_DEG && power > 0) {
            power = 0; // past upper limit — don't push further positive
        } else if (posDeg < TURRET_TARGET_DEG - TURRET_LIMIT_DEG && power < 0) {
            power = 0; // past lower limit — don't push further negative
        }

        turret.setPower(power);
    }

    // ======================================================================
    //  STATE MACHINE
    // ======================================================================
    public void statePathUpdate() {
        // Always run turret P-control
        updateTurret();

        boolean wantIntakeMotor = false;

        switch (pathState) {

            // ==== DRIVE TO SHOOT POSITION ====
            case DRIVE_STARTPOSE_TO_SHOOTPOSE:
                if (!pathStarted) {
                    follower.followPath(driveStartPoseToShootPose, true);
                    spindexer.setPosition(0.02);
                    hood.setPosition(autoHoodPos);
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

            // ==== SHOOT PRELOADED BALLS ====
            case SHOOT_PRELOAD:
                if (!shootTriggered) {
                    sdx.setShootRpm(autoShootRpm);
                    sdx.updateShoot(true, false, flywheel);
                    shootTriggered = true;
                    wasShootingLast = true;
                } else {
                    sdx.updateShoot(false, false, flywheel);
                }

                boolean currentlyShooting = sdx.isShooting();
                if (wasShootingLast && !currentlyShooting) {
                    pathState = PathState.DRIVE_SHOOTPOSE_TO_INTAKE1;
                    pathStarted = false;
                    shooterStarted = false;
                    break;
                }
                wasShootingLast = currentlyShooting || wasShootingLast;

                if (shooterTimer.seconds() >= 8) {
                    shootAttempts++;
                    if (shootAttempts >= 3) {
                        // 3 failures — skip intake to avoid penalties
                        flywheel.setPower(0);
                        sdx.resetShootState();
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

                telemetry.addLine("SHOOTING PRELOAD (attempt " + (shootAttempts + 1) + "/3)");
                break;

            // ==== DRIVE TO INTAKE 1 (half speed, intake motor ON) ====
            case DRIVE_SHOOTPOSE_TO_INTAKE1:
                if (!pathStarted) {
                    follower.followPath(driveShootPoseToIntake1, 0.5, true);
                    flywheel.setPower(0);
                    pathStarted = true;
                }
                wantIntakeMotor = true;

                if (pathStarted && !follower.isBusy()) {
                    pathState = PathState.DRIVE_INTAKE1_TO_SHOOTPOSE;
                    pathStarted = false;
                }
                break;

            // ==== DRIVE BACK TO SHOOT WITH BALL ====
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
                    hood.setPosition(autoHoodPos);
                }
                break;

            // ==== SHOOT INTAKE 1 BALLS ====
            case SHOOT_INTAKE1:
                if (!shootTriggered) {
                    sdx.setShootRpm(autoShootRpm);
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

                if (shooterTimer.seconds() >= 8) {
                    shootAttempts++;
                    if (shootAttempts >= 3) {
                        // 3 failures — skip intake to avoid penalties
                        flywheel.setPower(0);
                        sdx.resetShootState();
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

                telemetry.addLine("SHOOTING INTAKE 1 (attempt " + (shootAttempts + 1) + "/3)");
                break;

            // ==== DRIVE TO INTAKE 2 AREA (intake ON) ====
            case DRIVE_SHOOTPOSE_TO_INTAKEPOSE2:
                if (!pathStarted) {
                    follower.followPath(driveShootPoseToIntakePose2, true);
                    flywheel.setPower(0);
                    pathStarted = true;
                }
                wantIntakeMotor = true;

                if (pathStarted && !follower.isBusy()) {
                    pathState = PathState.DRIVE_INTAKEPOSE2_TO_INTAKE2;
                    pathStarted = false;
                }
                break;

            // ==== SLOW DRIVE THROUGH INTAKE 2 (half speed, intake ON) ====
            case DRIVE_INTAKEPOSE2_TO_INTAKE2:
                if (!pathStarted) {
                    follower.followPath(driveIntakePose2ToIntake2, 0.5, true);
                    pathStarted = true;
                }
                wantIntakeMotor = true;

                if (pathStarted && !follower.isBusy()) {
                    pathState = PathState.DRIVE_INTAKE2_TO_SHOOTPOSE;
                    pathStarted = false;
                }
                break;

            // ==== DRIVE BACK WITH INTAKE 2 BALL ====
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
                    hood.setPosition(autoHoodPos);
                }
                break;

            // ==== SHOOT INTAKE 2 BALLS ====
            case SHOOT_INTAKE2:
                if (!shootTriggered) {
                    sdx.setShootRpm(autoShootRpm);
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
                    break;
                }
                wasShootingLast = currentlyShooting || wasShootingLast;

                if (shooterTimer.seconds() >= 8) {
                    shootAttempts++;
                    if (shootAttempts >= 3) {
                        flywheel.setPower(0);
                        sdx.resetShootState();
                        pathState = PathState.DRIVE_SHOOTPOSE_TO_ENDPOSE;
                        pathStarted = false;
                    } else {
                        // Retry shoot — slot tracking from RPM drops preserved
                        sdx.resetShootState();
                        shootTriggered = false;
                        wasShootingLast = false;
                        shooterTimer.reset();
                    }
                }

                telemetry.addLine("SHOOTING INTAKE 2 (attempt " + (shootAttempts + 1) + "/3)");
                break;

            // ==== PARK ====
            case DRIVE_SHOOTPOSE_TO_ENDPOSE:
                if (!pathStarted) {
                    follower.followPath(driveShootPoseToEndPose, true);
                    flywheel.setPower(0);
                    intakeMotor.setPower(0);
                    turret.setPower(0);
                    pathStarted = true;
                }

                if (pathStarted && !follower.isBusy()) {
                    telemetry.addLine("Auto Complete");
                    turret.setPower(0);
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

        // Lookup RPM + hood from ShooterLookup table
        ShooterLookup.Result shootData = ShooterLookup.lookup(AUTO_SHOOT_DISTANCE);
        autoShootRpm = shootData.rpm;
        autoHoodPos  = shootData.hoodPos;

        // Initialize Pedro Follower (handles drive motors)
        follower = Constants.createFollower(hardwareMap);

        // Initialize servos (hood, flickers, spindexer)
        intServos(this);

        // Initialize sensors (color sensors, mag sensor)
        initSensors(this);

        // Initialize non-drive motors directly (avoid conflicting with Follower's drive motors)
        turret = hardwareMap.get(DcMotorEx.class, "turret");
        turret.setDirection(DcMotor.Direction.FORWARD);
        turret.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        turret.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        turret.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        flywheel = hardwareMap.get(DcMotorEx.class, "flywheel");
        flywheel.setDirection(DcMotor.Direction.REVERSE);
        flywheel.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        flywheel.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        intakeMotor = hardwareMap.get(DcMotorEx.class, "intake");
        intakeMotor.setDirection(DcMotor.Direction.FORWARD);
        intakeMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // SpindexerController
        sdx = new SpindexerController();
        sdx.setFlickerPositions(0.1, 0.0875, 0.5, 0.5);
        sdx.setShootRpm(autoShootRpm);
        sdx.prefillAllSlots();  // auto starts with 3 preloaded balls
        flicker1.setPosition(0.1);
        flicker2.setPosition(0.0875);
        spindexer.setPosition(SpindexerController.P1);

        // Set hood for shoot distance
        hood.setPosition(autoHoodPos);

        buildPaths();
        follower.setPose(startPose);

        telemetry.addData("Status", "Red Front Auto — Ready");
        telemetry.addData("Turret Target", TURRET_TARGET_DEG + "°");
        telemetry.addData("Shoot Distance", AUTO_SHOOT_DISTANCE + " in");
        telemetry.addData("Shoot RPM (lookup)", String.format("%.0f", autoShootRpm));
        telemetry.addData("Hood Pos (lookup)", String.format("%.3f", autoHoodPos));
        telemetry.update();
    }

    @Override
    public void init_loop() {
        // All 4 face buttons pressed → mark spindexer empty
        if (gamepad1.a && gamepad1.b && gamepad1.x && gamepad1.y) {
            sdx.clearAllSlots();
        }

        // Show spindexer state during init
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
        follower.update();

        statePathUpdate();

        // Telemetry
        telemetry.addData("Path state", pathState.toString());
        telemetry.addData("Shooting?", sdx.isShooting());
        telemetry.addData("Turret", String.format("%.1f°", turret.getCurrentPosition() / TICKS_PER_DEG));
        telemetry.addData("Flywheel", String.format("%.0f RPM", flywheel.getVelocity() * 60.0 / 28.0));
        telemetry.addData("Balls Loaded", sdx.getBallCount());
        telemetry.addData("Follower busy", follower.isBusy());
        telemetry.addData("X", follower.getPose().getX());
        telemetry.addData("Y", follower.getPose().getY());
        telemetry.addData("Heading", Math.toDegrees(follower.getPose().getHeading()));
        sdx.addTelemetry(telemetry);
        telemetry.update();
    }
}
