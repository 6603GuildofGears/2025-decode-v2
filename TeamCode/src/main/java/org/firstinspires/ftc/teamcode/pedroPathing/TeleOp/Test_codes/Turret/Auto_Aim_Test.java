package org.firstinspires.ftc.teamcode.pedroPathing.TeleOp.Test_codes.Turret;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.pedroPathing.Pipelines.TurretPipeline;

import static org.firstinspires.ftc.teamcode.pedroPathing.Pipelines.Motor_PipeLine.*;
import static org.firstinspires.ftc.teamcode.pedroPathing.Pipelines.Limelight_Pipeline.*;
import static org.firstinspires.ftc.teamcode.pedroPathing.Pipelines.Sensor.*;

/**
 * Simple auto-aim test using the TurretPipeline PID controller.
 * Drive with gamepad1, turret auto-tracks the blue goal AprilTag.
 */
@Disabled
@TeleOp(name = "Auto Aim Test", group = "Testing")
public class Auto_Aim_Test extends OpMode {

    private TurretPipeline turretPID;

    @Override
    public void init() {
        intMotors(this);
        initLimelight(this);
        initSensors(this);

        turret.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        turret.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        turretPID = new TurretPipeline();

        telemetry.addData("Status", "Initialized");
        telemetry.update();
    }

    @Override
    public void start() {
        turretPID.resetTimer();
    }

    @Override
    public void loop() {
        // --- Drive (simple mecanum) ---
        double drive  = gamepad1.left_stick_y;
        double strafe = -gamepad1.left_stick_x;
        double turn   = -gamepad1.right_stick_x;
        double gear   = 1.25;

        if (Math.abs(drive) > 0.05 || Math.abs(strafe) > 0.05 || Math.abs(turn) > 0.05) {
            double lf = (drive + strafe + turn) * gear;
            double lb = (drive - strafe + turn) * gear;
            double rf = (drive - strafe - turn) * gear;
            double rb = (drive + strafe - turn) * gear;

            double max = Math.max(Math.abs(lf),
                         Math.max(Math.abs(lb),
                         Math.max(Math.abs(rf), Math.abs(rb))));
            if (max > 1.0) { lf /= max; lb /= max; rf /= max; rb /= max; }

            frontLeft.setPower(lf);
            backLeft.setPower(lb);
            frontRight.setPower(rf);
            backRight.setPower(rb);
        } else {
            SetPower(0, 0, 0, 0);
        }

        // --- Turret auto-aim ---
        boolean hasTarget = hasBlueGoal();
        double tx = hasTarget ? getBlueGoalX() : 0;

        turretPID.update(hasTarget, tx);

        // --- Telemetry ---
        telemetry.addData("=== AUTO AIM TEST ===", "");
        telemetry.addData("Target", hasTarget ? "BLUE LOCKED" : "SCANNING");
        telemetry.addData("TX", hasTarget ? String.format("%.2f°", tx) : "--");
        telemetry.addData("Turret Power", String.format("%.3f", turretPID.getPower()));
        telemetry.addData("Turret Pos", turret.getCurrentPosition());
        telemetry.addData("kP", String.format("%.4f", turretPID.getKP()));
        telemetry.addData("kI", String.format("%.4f", turretPID.getKI()));
        telemetry.addData("kD", String.format("%.4f", turretPID.getKD()));
        telemetry.update();
    }
}