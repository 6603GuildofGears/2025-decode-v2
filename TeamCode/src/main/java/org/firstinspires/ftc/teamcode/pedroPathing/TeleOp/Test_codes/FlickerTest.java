package org.firstinspires.ftc.teamcode.pedroPathing.TeleOp.Test_codes;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;

@Disabled
@TeleOp(name = "Flicker Test", group = "Test")
public class FlickerTest extends LinearOpMode {

    @Override
    public void runOpMode() {
        Servo flicker = hardwareMap.servo.get("flicker");
        DcMotorEx flywheel = hardwareMap.get(DcMotorEx.class, "flywheel");

        flicker.setPosition(0);
        flywheel.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        flywheel.setDirection(DcMotor.Direction.REVERSE);

        telemetry.addData("Status", "Ready — press START");
        telemetry.update();
        waitForStart();

        // Spin up flywheel (ticks/sec = RPM * 28 / 60)
        double targetRPM = 3000;
        flywheel.setVelocity(targetRPM * 28.0 / 60.0);

        while (opModeIsActive()) {
            if (gamepad1.a) {
                flicker.setPosition(0.4);
            } else {
                flicker.setPosition(0);
            }

            telemetry.addData("Flicker Pos", flicker.getPosition());
            telemetry.addData("Flywheel RPM", "%.0f", flywheel.getVelocity() * 60.0 / 28.0);
            telemetry.addData("A", "Flicker to 0.4");
            telemetry.addData("B", "Flicker to 0");
            telemetry.update();
        }

        flywheel.setPower(0);
    }
}
