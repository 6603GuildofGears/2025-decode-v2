package org.firstinspires.ftc.teamcode.pedroPathing.TeleOp.Test_codes;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

/**
 * MOTOR DIRECTION DIAGNOSTIC TEST
 * ================================
 * Put the robot on a box so ALL wheels are off the ground.
 * 
 * This test spins ONE motor at a time with POSITIVE power (+0.3).
 * 
 * Use D-pad to select which motor to test:
 *   D-pad UP    = frontLeft
 *   D-pad RIGHT = frontRight
 *   D-pad DOWN  = backRight
 *   D-pad LEFT  = backLeft
 * 
 * Press A = spin selected motor FORWARD (positive power)
 * Press B = spin selected motor REVERSE (negative power)
 * 
 * Release the button to stop.
 * 
 * WHAT TO REPORT FOR EACH MOTOR:
 * 1. Which physical wheel actually spins? (front-left, front-right, back-left, back-right)
 * 2. When pressing A, does the wheel spin so the robot would go FORWARD or BACKWARD?
 *    (Look at the top of the wheel - if it spins toward the front of the robot = forward)
 * 
 * This uses the SAME motor names and directions as Pedro Pathing Constants.java
 */
@TeleOp(name="!! Motor Direction Test !!", group="Test")
public class MotorDirectionTest extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {

        // Use the same hardware map names as Pedro Pathing Constants.java
        DcMotorEx frontLeft  = hardwareMap.get(DcMotorEx.class, "frontLeft");
        DcMotorEx frontRight = hardwareMap.get(DcMotorEx.class, "frontRight");
        DcMotorEx backLeft   = hardwareMap.get(DcMotorEx.class, "backLeft");
        DcMotorEx backRight  = hardwareMap.get(DcMotorEx.class, "backRight");

        // Set the SAME directions as Pedro Pathing Constants.java
        frontLeft.setDirection(DcMotor.Direction.REVERSE);   // leftFrontMotorDirection
        frontRight.setDirection(DcMotor.Direction.FORWARD);  // rightFrontMotorDirection
        backLeft.setDirection(DcMotor.Direction.REVERSE);    // leftRearMotorDirection
        backRight.setDirection(DcMotor.Direction.REVERSE);   // rightRearMotorDirection

        frontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        String selectedMotor = "NONE";
        double testPower = 0.3;

        telemetry.addLine("=== MOTOR DIRECTION TEST ===");
        telemetry.addLine("Robot must be OFF THE GROUND!");
        telemetry.addLine("");
        telemetry.addLine("D-pad to select motor");
        telemetry.addLine("A = spin positive / B = spin negative");
        telemetry.addLine("");
        telemetry.addLine("Ready - press Play");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {

            // Select motor with D-pad
            if (gamepad1.dpad_up)    selectedMotor = "frontLeft";
            if (gamepad1.dpad_right) selectedMotor = "frontRight";
            if (gamepad1.dpad_left)  selectedMotor = "backLeft";
            if (gamepad1.dpad_down)  selectedMotor = "backRight";

            // Stop all motors first
            frontLeft.setPower(0);
            frontRight.setPower(0);
            backLeft.setPower(0);
            backRight.setPower(0);

            // Determine power direction
            double power = 0;
            String direction = "STOPPED";
            if (gamepad1.a) {
                power = testPower;
                direction = "POSITIVE (+)";
            } else if (gamepad1.b) {
                power = -testPower;
                direction = "NEGATIVE (-)";
            }

            // Apply power to selected motor only
            if (power != 0) {
                switch (selectedMotor) {
                    case "frontLeft":  frontLeft.setPower(power);  break;
                    case "frontRight": frontRight.setPower(power); break;
                    case "backLeft":   backLeft.setPower(power);   break;
                    case "backRight":  backRight.setPower(power);  break;
                }
            }

            // Telemetry
            telemetry.addLine("╔══════════════════════════════╗");
            telemetry.addLine("║   MOTOR DIRECTION TEST       ║");
            telemetry.addLine("╚══════════════════════════════╝");
            telemetry.addLine("");
            telemetry.addData("Selected Motor", selectedMotor);
            telemetry.addData("Power", direction);
            telemetry.addLine("");
            telemetry.addLine("--- SELECT WITH D-PAD ---");
            telemetry.addData("  D-pad UP",    selectedMotor.equals("frontLeft")  ? ">>> frontLeft <<<" : "frontLeft");
            telemetry.addData("  D-pad RIGHT", selectedMotor.equals("frontRight") ? ">>> frontRight <<<" : "frontRight");
            telemetry.addData("  D-pad LEFT",  selectedMotor.equals("backLeft")   ? ">>> backLeft <<<" : "backLeft");
            telemetry.addData("  D-pad DOWN",  selectedMotor.equals("backRight")  ? ">>> backRight <<<" : "backRight");
            telemetry.addLine("");
            telemetry.addLine("Hold A = positive power");
            telemetry.addLine("Hold B = negative power");
            telemetry.addLine("");
            telemetry.addLine("--- REPORT BACK ---");
            telemetry.addLine("1. Which PHYSICAL wheel spins?");
            telemetry.addLine("2. With A pressed, does the wheel");
            telemetry.addLine("   spin FORWARD or BACKWARD?");
            telemetry.addLine("   (top of wheel toward front = fwd)");
            telemetry.update();
        }
    }
}
