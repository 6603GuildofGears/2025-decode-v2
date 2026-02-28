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
 * Simplified Red Back Auto - Just shoot preload and park
 */
@Autonomous(name = "Red Back Preload", group = "Red")
public class Red_Back_Preload extends OpMode {
    private Follower follower;
    private Timer pathTimer, opmodeTimer;
    private SpindexerController spindexer;
    private DcMotorEx flywheel;
    private DcMotorEx intake;
    private DcMotorEx intakeMotor2;
    private DcMotorEx turret;
    private Motor_PipeLine motorPipeline;
    private Servo_Pipeline servoPipeline;
    private boolean shootSequenceStarted = false;
    private boolean scanningInProgress = false;

    // Shooter settings
    private double shootRpm = 4100;
    private double shootHood = 0.5;
    private static final int GOAL_TAG_ID = 10;

    // Turret
    private static final double TURRET_TARGET_DEG = 75;
    private static final double TURRET_MOTIF_SCAN_DEG = 75;
    private static final double TURRET_P_GAIN = 0.006;
    private static final double TURRET_MAX_POWER = 0.30;
    private static final double TURRET_LIMIT_DEG = 5.0;
    
    private static final double MOTIF_SCAN_TIME_SEC = 1.8;
    private ElapsedTime motifScanTimer = new ElapsedTime();
    private boolean scanningForMotif = false;
    private boolean turretAtShootAngle = false;
    
    // Motif
    private int detectedTagId = -1;
    private String[] motifOrder = null;
    private boolean motifLocked = false;

    public enum PathState {
        SHOOT_PRELOAD,
        DRIVE_SHOOT_TO_END
    }

    PathState pathState;

    // Simple poses - start (shoot here) and end (park)
    private final Pose startPose = new Pose(87, 9, Math.toRadians(90));
    private final Pose endPose = new Pose(101, 12, Math.toRadians(90));

    private boolean pathStarted = false;

    private PathChain driveShootToEnd;

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
        driveShootToEnd = follower.pathBuilder()
                .addPath(new BezierLine(startPose, endPose))
                .setConstantHeadingInterpolation(endPose.getHeading())
                .build();
    }

    public void statePathUpdate() {
        updateTurret();

        switch (pathState) {
            case SHOOT_PRELOAD:
                if (!shootSequenceStarted) {
                    // Spin up flywheel
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
                    intake.setPower(-0.5);
                    intakeMotor2.setPower(1.0);
                } else {
                    spindexer.updateShoot(false, false, flywheel);
                }

                if (!spindexer.isShooting() && readyToShoot) {
                    intake.setPower(0);
                    intakeMotor2.setPower(0);
                    flywheel.setVelocity(0);
                    shootSequenceStarted = false;
                    spindexer.resetShootState();
                    spindexer.clearAllSlots();
                    Servo_Pipeline.flicker.setPosition(0);

                    pathState = PathState.DRIVE_SHOOT_TO_END;
                    pathStarted = false;
                }
                telemetry.addLine("Shooting preload");
                telemetry.addData("Turret", turretAtShootAngle ? "READY" : "Turning...");
                telemetry.addData("Motif", scanningForMotif ? "Scanning..." : (motifLocked ? Motif.getMotifName(detectedTagId) : "FALLBACK"));
                telemetry.addData("Confirmed", spindexer.getConfirmedShots());
                break;

            case DRIVE_SHOOT_TO_END:
                if (!pathStarted) {
                    follower.followPath(driveShootToEnd, true);
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

        telemetry.addData("Status", "Initialized - Simple Mode");
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
        spindexer.updateIntake(false);  // No intaking

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
