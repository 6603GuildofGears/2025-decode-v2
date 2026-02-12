
package org.firstinspires.ftc.teamcode.pedroPathing.TeleOp.Test_codes;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

/**
 * Bare-minimum turret encoder test.
 * No pipelines, no follower, no PID, no vision.
 * Just reads the encoder and shows it.
 *
 * If ticks don't change here, it's a hardware/wiring issue:
 *   - Encoder cable not plugged in (separate from motor power on goBILDA)
 *   - Encoder cable on wrong port
 *   - Encoder cable damaged
 *
 * Controls: gamepad2 left stick X = spin turret at 30% power
 */
@TeleOp(name = "!! Encoder Diag", group = "Testing")
public class EncoderDiag extends OpMode {

    private DcMotorEx turretMotor;

    @Override
    public void init() {
        turretMotor = hardwareMap.get(DcMotorEx.class, "turret");
        turretMotor.setDirection(DcMotor.Direction.FORWARD);
        turretMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        turretMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        turretMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        telemetry.addData("Status", "Ready — use GP2 left stick X to spin");
        telemetry.update();
    }

    @Override
    public void loop() {
        // Drive turret with stick
        double stick = gamepad2.left_stick_x;
        if (Math.abs(stick) > 0.1) {
            turretMotor.setPower(stick * 0.3);
        } else {
            turretMotor.setPower(0);
        }

        int ticks = turretMotor.getCurrentPosition();

        telemetry.addData("TICKS", ticks);
        telemetry.addData("ANGLE (raw)", String.format("%.1f", ticks / 2.65));
        telemetry.addData("POWER", String.format("%.2f", stick * 0.3));
        telemetry.addData("MODE", turretMotor.getMode());
        telemetry.addData("", "");
        telemetry.addData("Motor spins?", "If YES but ticks=0, encoder cable issue");
        telemetry.addData("Check", "1. Encoder cable plugged into SAME port as motor");
        telemetry.addData("Check", "2. Cable not damaged / loose");
        telemetry.addData("Check", "3. Try swapping to a different hub port");
        telemetry.update();
    }
}
