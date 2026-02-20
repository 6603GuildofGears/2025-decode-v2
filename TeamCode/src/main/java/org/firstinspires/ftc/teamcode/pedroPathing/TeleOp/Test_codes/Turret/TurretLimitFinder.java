package org.firstinspires.ftc.teamcode.pedroPathing.TeleOp.Test_codes.Turret;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

@Disabled
 

@TeleOp(name="Turret Limit Finder", group="Testing")
public class TurretLimitFinder extends LinearOpMode {

    private DcMotorEx turret;

    @Override 
    public void runOpMode() throws InterruptedException {

        // Initialize turret motor directly
        turret = hardwareMap.get(DcMotorEx.class, "turret");
        turret.setDirection(DcMotor.Direction.FORWARD);
        turret.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        
        // Reset encoder to 0 at start
        turret.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        turret.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        telemetry.addData("Status", "Turret encoder reset to 0");
        telemetry.addData("Instructions", "Use Left Stick X on gamepad 2");
        telemetry.addData("Note", "Position should change as you rotate");
        telemetry.update();

        waitForStart();

        // Variables to track min and max positions seen
        int minPositionSeen = 0;
        int maxPositionSeen = 0;

        while (opModeIsActive()) {

            // Get gamepad input
            double LStickX2 = gamepad2.left_stick_x;

            // Control turret with left stick
            if (Math.abs(LStickX2) > 0.1) {
                turret.setPower(LStickX2 * 0.5); // Half power for careful movement
            } else {
                turret.setPower(0);
            }

            // Get current turret position
            int turretPosition = turret.getCurrentPosition();

            // Track min and max positions
            if (turretPosition < minPositionSeen) {
                minPositionSeen = turretPosition;
            }
            if (turretPosition > maxPositionSeen) {
                maxPositionSeen = turretPosition;
            }

            // Display encoder position and instructions
            telemetry.addData("=== TURRET LIMIT FINDER ===", "");
            telemetry.addData("Current Position", turretPosition);
            telemetry.addData("", "");
            telemetry.addData("Min Position Seen", minPositionSeen);
            telemetry.addData("Max Position Seen", maxPositionSeen);
            telemetry.addData("", "");
            telemetry.addData("Instructions:", "");
            telemetry.addData("1.", "Rotate turret LEFT until wire tension");
            telemetry.addData("2.", "Note the Min Position value");
            telemetry.addData("3.", "Rotate turret RIGHT until wire tension");
            telemetry.addData("4.", "Note the Max Position value");
            telemetry.addData("5.", "Add these to Cassius.java:");
            telemetry.addData("   turretMinLimit", "= " + minPositionSeen);
            telemetry.addData("   turretMaxLimit", "= " + maxPositionSeen);
            telemetry.addData("   limitsEnabled", "= true");
            telemetry.update();
        }
    }
}
