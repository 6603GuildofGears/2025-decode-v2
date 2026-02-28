package org.firstinspires.ftc.teamcode.pedroPathing.Autos.Motif;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.pedropathing.util.Timer;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.*;
import com.pedropathing.paths.*;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.pedroPathing.other.Constants;
import org.firstinspires.ftc.teamcode.pedroPathing.Pipelines.Limelight_Pipeline;
import org.firstinspires.ftc.teamcode.pedroPathing.Pipelines.Motor_PipeLine;
import org.firstinspires.ftc.teamcode.pedroPathing.Pipelines.Motif;
import org.firstinspires.ftc.teamcode.pedroPathing.Pipelines.MotifQueue;
import org.firstinspires.ftc.teamcode.pedroPathing.Pipelines.SpindexerController;
import org.firstinspires.ftc.teamcode.pedroPathing.Pipelines.Servo_Pipeline;
import org.firstinspires.ftc.teamcode.pedroPathing.Pipelines.Sensor;

import static org.firstinspires.ftc.teamcode.pedroPathing.TeleOp.TurretConfig.*;

/**
 * Blue Back Auto with Motif detection - Simplified with single intake cycle
 * Start at 90° heading, turret at 120°, one intake cycle before parking
 */
@Autonomous(name = "Blue Back cre8 playoffs", group = "Blue")
public class Blue_Back_Cre8 extends OpMode {
    private Follower follower;
    private Timer pathTimer, opmodeTimer;
    private SpindexerController spindexer;
    private DcMotorEx flywheel;
    private DcMotorEx intake;
    private DcMotorEx intakeMotor2;
    private DcMotorEx turret;
    private Motor_PipeLine motorPipeline;
    private Servo_Pipeline servoPipeline;
    private boolean intakeRunning = false;
    private boolean shootSequenceStarted = false;
    private boolean scanningInProgress = false;

    // Shooter settings
    private double shootRpm = 4150;
    private double shootHood = 0.55;
    private static final int GOAL_TAG_ID = 20;

    // Turret - BLUE BACK ANGLES
    private static final double TURRET_TARGET_DEG = 30;
    private static final double TURRET_MOTIF_SCAN_DEG = 60;
    private static final double TURRET_P_GAIN = 0.006;
    private static final double TURRET_MAX_POWER = 0.30;
    private static final double TURRET_LIMIT_DEG = 5.0;
    
    private static final double MOTIF_SCAN_TIME_SEC = 1.8;
    private ElapsedTime motifScanTimer = new ElapsedTime();
    private boolean scanningForMotif = false;
    private boolean turretAtShootAngle = false;

    
    // Motif detection
    private int detectedTagId = -1;
    private String[] motifOrder = null;
    private boolean motifLocked = false;

    public enum PathState {
        SHOOT_PRELOAD,
        DRIVE_TO_INTAKE2,
        DRIVE_INTAKE2_TO_SHOOTPOSE,
        SHOOT_INTAKE2,
        DRIVE_TO_ENDPOSE
    }

    PathState pathState;

    // Poses - mirrored from Red_Back with 90° heading
    private final Pose startPose = new Pose(57, 9, Math.toRadians(90));
    private final Pose intake1 = new Pose(8, 20, Math.toRadians(225));
    private final Pose intake2 = new Pose(8, 10, Math.toRadians(225));
    private final Pose endPose = new Pose(44, 12, Math.toRadians(90));

    private boolean pathStarted = false;

    private PathChain driveToIntake2;
    private PathChain driveIntake2ToShootPose;
    private PathChain driveToEndPose;

    private void updateTurret() {
        double targetDeg = scanningForMotif ? TURRET_MOTIF_SCAN_DEG : TURRET_TARGET_DEG;
            
        int pos = turret.getCurrentPosition();
        double posDeg = pos / TICKS_PER_DEG;
        double targetTicks = targetDeg * TICKS_PER_DEG;
        double error = targetTicks - pos;
        double power = error * TURRET_P_GAIN;
        power = Math.max(-TURRET_MAX_POWER, Math.min(TURRET_MAX_POWER, power));
        
        if (posDeg > targetDeg + TURRET_LIMIT_DEG && power > 0) power = 0;
        else if (posDeg < targetDeg - TURRET_LIMIT_DEG && power < 0) power = 0;
        
        turret.setPower(power);
        turretAtShootAngle = (Math.abs(posDeg - TURRET_TARGET_DEG) <= TURRET_LIMIT_DEG);
    }

    private double getTickSpeed(double rpm) { return rpm * 28.0 / 60.0; }

    public void buildPaths() {
        driveToIntake2 = follower.pathBuilder()
                .addPath(new BezierLine(startPose, intake1))
                .setLinearHeadingInterpolation(startPose.getHeading(), intake1.getHeading())
                .addPath(new BezierLine(intake1, intake2))
                .setConstantHeadingInterpolation(intake2.getHeading())
                .build();

        driveIntake2ToShootPose = follower.pathBuilder()
                .addPath(new BezierLine(intake2, startPose))
                .setLinearHeadingInterpolation(intake2.getHeading(), startPose.getHeading())
                .build();

        driveToEndPose = follower.pathBuilder()
                .addPath(new BezierLine(startPose, endPose))
                .setConstantHeadingInterpolation(endPose.getHeading())
                .build();
    }

    public void statePathUpdate() {
        updateTurret();

        switch (pathState) {
            case SHOOT_PRELOAD:
                if (!shootSequenceStarted) {
                    // Spin up flywheel and start scanning
                    Servo_Pipeline.spindexerAxon.setTargetRotation(SpindexerController.P1);
                    flywheel.setVelocity(getTickSpeed(shootRpm));
                    
                    spindexer.startScan("GREEN");
                    scanningInProgress = true;
                    
                    scanningForMotif = true;
                    motifScanTimer.reset();
                    
                    shootSequenceStarted = true;
                }
                
                // Scan for motif
                if (scanningForMotif) {
                    if (!motifLocked) {
                        Limelight_Pipeline.pollOnce();
                        int tagId = Limelight_Pipeline.getObeliskTagId();
                        if (tagId != -1) {
                            detectedTagId = tagId;
                            motifOrder = Motif.getShootOrder(tagId);
                            motifLocked = true;
                        }
                    }
                    
                    if (motifLocked || motifScanTimer.seconds() >= MOTIF_SCAN_TIME_SEC) {
                        scanningForMotif = false;
                        if (!motifLocked) {
                            motifOrder = Motif.getShootOrder(-1);
                        }
                    }
                }
                
                // Scan balls
                if (scanningInProgress && !spindexer.isScanDone()) {
                    spindexer.updateScan();
                } else if (scanningInProgress && spindexer.isScanDone()) {
                    scanningInProgress = false;
                    
                    // Build shoot queue
                    if (motifOrder != null) {
                        int[] queue = MotifQueue.buildMotifQueue(
                            motifOrder,
                            spindexer.getSlotColors(),
                            spindexer.getSlotEmptyStatus()
                        );
                        spindexer.setShootQueue(queue);
                    }
                }
                
                // Ready to shoot when turret is at angle and scan is done
                boolean readyToShoot = turretAtShootAngle && !scanningInProgress;
                
                if (readyToShoot) {
                    spindexer.updateShoot(true, false, flywheel);
                    intake.setPower(-0.75);
                    intakeMotor2.setPower(1.0);
                } else {
                    spindexer.updateShoot(false, false, flywheel);
                }

                if (!spindexer.isShooting() && readyToShoot) {
                    intake.setPower(0);
                    intakeMotor2.setPower(0);
                    flywheel.setVelocity(0);
                    spindexer.resetShootState();
                    spindexer.clearAllSlots();
                    Servo_Pipeline.flicker.setPosition(0);

                    pathState = PathState.DRIVE_TO_INTAKE2;
                    pathStarted = false;
                }
                telemetry.addLine("Shooting preload");
                telemetry.addData("Turret", turretAtShootAngle ? "READY" : "Turning...");
                telemetry.addData("Motif", scanningForMotif ? "Scanning..." : (motifLocked ? Motif.getMotifName(detectedTagId) : "FALLBACK"));
                telemetry.addData("Confirmed", spindexer.getConfirmedShots());
                break;

            case DRIVE_TO_INTAKE2:
                if (!pathStarted) {
                    follower.followPath(driveToIntake2, true);
                    pathStarted = true;
                }
                
                // Run intake during drive
                if (!intakeRunning) {
                    intake.setPower(0.5);
                    intakeMotor2.setPower(0.5);
                    spindexer.startScan("GREEN");
                    scanningInProgress = true;
                    intakeRunning = true;
                }
                
                // Update ball scanning
                if (scanningInProgress && !spindexer.isScanDone()) {
                    spindexer.updateScan();
                } else if (scanningInProgress && spindexer.isScanDone()) {
                    scanningInProgress = false;
                }
                
                if (pathStarted && !follower.isBusy()) {
                    intake.setPower(0);
                    intakeMotor2.setPower(0);
                    intakeRunning = false;
                    
                    pathState = PathState.DRIVE_INTAKE2_TO_SHOOTPOSE;
                    pathStarted = false;
                }
                telemetry.addLine("Driving to intake 2");
                break;

            case DRIVE_INTAKE2_TO_SHOOTPOSE:
                if (!pathStarted) {
                    follower.followPath(driveIntake2ToShootPose, true);
                    flywheel.setVelocity(getTickSpeed(shootRpm));
                    pathStarted = true;
                }
                
                if (pathStarted && !follower.isBusy() && turretAtShootAngle) {
                    // Build shoot queue if we have motif order
                    if (motifOrder != null) {
                        int[] queue = MotifQueue.buildMotifQueue(
                            motifOrder,
                            spindexer.getSlotColors(),
                            spindexer.getSlotEmptyStatus()
                        );
                        spindexer.setShootQueue(queue);
                    }
                    
                    pathState = PathState.SHOOT_INTAKE2;
                    pathStarted = false;
                    shootSequenceStarted = false;
                }
                telemetry.addLine("Returning to shoot");
                break;

            case SHOOT_INTAKE2:
                if (!shootSequenceStarted) {
                    spindexer.updateShoot(true, false, flywheel);
                    intake.setPower(-0.5);
                    intakeMotor2.setPower(1.0);
                    shootSequenceStarted = true;
                } else {
                    spindexer.updateShoot(false, false, flywheel);
                }

                if (!spindexer.isShooting() && shootSequenceStarted) {
                    intake.setPower(0);
                    intakeMotor2.setPower(0);
                    flywheel.setVelocity(0);
                    shootSequenceStarted = false;
                    spindexer.resetShootState();
                    spindexer.clearAllSlots();
                    Servo_Pipeline.flicker.setPosition(0);

                    pathState = PathState.DRIVE_TO_ENDPOSE;
                    pathStarted = false;
                }
                telemetry.addLine("Shooting intake 2");
                telemetry.addData("Confirmed", spindexer.getConfirmedShots());
                break;

            case DRIVE_TO_ENDPOSE:
                if (!pathStarted) {
                    follower.followPath(driveToEndPose, true);
                    pathStarted = true;
                }
                if (pathStarted && !follower.isBusy()) {
                    turret.setPower(0);
                    requestOpModeStop();
                }
                telemetry.addLine("Parking...");
                break;
        }
    }

    @Override
    public void init() {
        pathState = PathState.SHOOT_PRELOAD;
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
        spindexer = new SpindexerController();
        spindexer.setFlickerPositions(0, 0.375);
        spindexer.setShootRpm(shootRpm);
        spindexer.prefillAllSlots();

        Limelight_Pipeline.initLimelight(this);

        buildPaths();
        follower.setPose(startPose);

        telemetry.addData("Status", "Initialized - Blue Back with intake cycle");
        telemetry.update();
    }

    @Override
    public void init_loop() {
        Limelight_Pipeline.pollOnce();
        int tagId = Limelight_Pipeline.getObeliskTagId();

        if (tagId != -1) {
            detectedTagId = tagId;
            motifOrder = Motif.getShootOrder(tagId);
            motifLocked = true;
        }

        telemetry.addData("Motif", motifLocked ? Motif.getMotifName(detectedTagId) : "Scanning...");
        telemetry.update();
    }

    @Override
    public void start() {
        Servo_Pipeline.flicker.setPosition(0);
        Servo_Pipeline.spindexerAxon.setTargetRotation(SpindexerController.P1);
        Servo_Pipeline.hood.setPosition(shootHood);
        opmodeTimer.resetTimer();
        pathState = PathState.SHOOT_PRELOAD;
        spindexer.goToSlot(0);

        if (motifLocked && motifOrder != null) {
            spindexer.setMotifOrder(motifOrder);
        }
    }

    @Override
    public void loop() {
        follower.update();
        Sensor.updateSensors();
        
        statePathUpdate();
        spindexer.updateIntake(false);

        telemetry.addData("Motif System", motifLocked ? "ON" : "OFF");
        telemetry.addData("Path", pathState.toString());
        
        double actualVelocity = flywheel.getVelocity();
        double actualRpm = actualVelocity * 60.0 / 28.0;
        telemetry.addData("Shooter RPM", String.format("%.0f / %.0f", actualRpm, shootRpm));
        
        telemetry.addData("X", follower.getPose().getX());
        telemetry.addData("Y", follower.getPose().getY());
        telemetry.update();
    }
}
