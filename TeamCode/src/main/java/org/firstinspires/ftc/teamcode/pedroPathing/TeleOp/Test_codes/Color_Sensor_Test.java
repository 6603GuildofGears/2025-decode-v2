package org.firstinspires.ftc.teamcode.pedroPathing.TeleOp.Test_codes;

import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.Servo;

import static org.firstinspires.ftc.teamcode.pedroPathing.Pipelines.Motor_PipeLine.*;
import static org.firstinspires.ftc.teamcode.pedroPathing.Pipelines.Servo_Pipeline.*;
import static org.firstinspires.ftc.teamcode.pedroPathing.Pipelines.Sensor.*;

/**
 * Simple test for color sensor + spindexer + intake.
 *
 * Controls:
 *   RT (gp1)     = intake in
 *   RB (gp1)     = intake out
 *   A  (gp1)     = spindexer to p1
 *   B  (gp1)     = spindexer to p2
 *   Y  (gp1)     = spindexer to p3
 *   dpad L/R     = nudge spindexer
 */
@Disabled
@TeleOp(name = "Color Sensor Test", group = "Testing")
public class Color_Sensor_Test extends LinearOpMode {

    static TelemetryManager telemetryM;

    @Override
    public void runOpMode() {
        intMotors(this);
        intServos(this);
        initSensors(this);
        telemetryM = PanelsTelemetry.INSTANCE.getTelemetry();

        double p1 = 0.0;
        double p2 = 0.375;
        double p3 = 0.75;
        double slewSpeed = 0.025; // how much the servo moves per loop (~50hz → full travel in ~0.6s)
        double targetPos = p1;

        int currentSlot = 1; // 1-3
        String[] slotColors = {"NONE", "NONE", "NONE"};
        boolean[] slotLoaded = {false, false, false};

        spindexer.setDirection(Servo.Direction.REVERSE);
        spindexer.setPosition(p1);
        currentSlot = 1;

        // Flicker to rest
        flicker.setPosition(0.1);

        telemetry.addData("Status", "Ready — press START");
        telemetry.addData("", "");
        telemetry.addData("=== CONTROLS ===", "");
        telemetry.addData("A", "Spindexer to p1");
        telemetry.addData("B", "Spindexer to p2");
        telemetry.addData("Y", "Spindexer to p3");
        telemetry.addData("X", "Clear all slots");
        telemetry.addData("RT", "Intake in");
        telemetry.addData("RB", "Intake reverse");
        telemetry.addData("Dpad L/R", "Nudge spindexer");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {
            // === Intake ===
            if (gamepad1.right_trigger > 0.1) {
                intake.setPower(0.75);
            } else if (gamepad1.right_bumper) {
                intake.setPower(-1);
            } else {
                intake.setPower(0);
            }

            // === Spindexer position (set target) ===
            if (gamepad1.a) {
                targetPos = p1;
                currentSlot = 1;
            } else if (gamepad1.b) {
                targetPos = p2;
                currentSlot = 2;
            } else if (gamepad1.y) {
                targetPos = p3;
                currentSlot = 3;
            }

            // Nudge (adjusts target)
            if (gamepad1.dpad_right) {
                targetPos = Math.min(1.0, targetPos + 0.005);
            } else if (gamepad1.dpad_left) {
                targetPos = Math.max(0.0, targetPos - 0.005);
            }

            // === Gradual slew toward target ===
            double currentPos = spindexer.getPosition();
            double error = targetPos - currentPos;
            if (Math.abs(error) > 0.001) {
                double step = Math.signum(error) * Math.min(slewSpeed, Math.abs(error));
                spindexer.setPosition(currentPos + step);
            }

            // === Color sensor — only read at actual slot positions, never override ===
            boolean atSlotPos = (Math.abs(spindexer.getPosition() - p1) < 0.005)
                             || (Math.abs(spindexer.getPosition() - p2) < 0.005)
                             || (Math.abs(spindexer.getPosition() - p3) < 0.005);
            String detectedColor = "---";
            boolean ballPresent = false;

            if (atSlotPos) {
                updateSensors();
                detectedColor = detectBallColor();
                ballPresent = isBallPresent();

                // Update slot with what sensor sees
                if (ballPresent) {
                    slotColors[currentSlot - 1] = detectedColor;
                    slotLoaded[currentSlot - 1] = true;
                } else {
                    slotColors[currentSlot - 1] = "NONE";
                    slotLoaded[currentSlot - 1] = false;
                }
            }

            // Clear ALL slots with X
            if (gamepad1.x) {
                for (int i = 0; i < 3; i++) {
                    slotColors[i] = "NONE";
                    slotLoaded[i] = false;
                }
            }

            // === Telemetry ===
            telemetry.addData("=== COLOR SENSOR ===", "");
            if (atSlotPos) {
                telemetry.addData("Ball Present", ballPresent ? "YES" : "no");
                telemetry.addData("Detected Color", detectedColor);
                telemetry.addData("ColorSensor", getColorSensorDebug());
            } else {
                telemetry.addData("Status", "MOVING...");
            }

            telemetry.addData("", "");
            telemetry.addData("=== SPINDEXER ===", "");
            telemetry.addData("Current Slot", "p" + currentSlot);
            telemetry.addData("Servo Pos", String.format("%.3f", spindexer.getPosition()));

            telemetry.addData("", "");
            telemetry.addData("=== SLOT TRACKING ===", "");
            telemetry.addData("p1", (slotLoaded[0] ? "● " : "○ ") + slotColors[0]);
            telemetry.addData("p2", (slotLoaded[1] ? "● " : "○ ") + slotColors[1]);
            telemetry.addData("p3", (slotLoaded[2] ? "● " : "○ ") + slotColors[2]);

            telemetry.addData("", "");
            telemetry.addData("Controls", "A=p1  B=p2  Y=p3  X=clear slot");
            telemetry.addData("", "RT=intake  RB=reverse  dpad=nudge");

            // === Panels telemetry ===
            if (atSlotPos) {
                telemetryM.debug("Ball Present: " + (ballPresent ? "YES" : "no"));
                telemetryM.debug("Color: " + detectedColor + " | " + getColorSensorDebug());
            } else {
                telemetryM.debug("MOVING...");
            }
            telemetryM.debug("Slot: p" + currentSlot + " | Servo: " + String.format("%.3f", spindexer.getPosition()));
            telemetryM.debug("p1: " + (slotLoaded[0] ? "● " : "○ ") + slotColors[0]
                    + "  p2: " + (slotLoaded[1] ? "● " : "○ ") + slotColors[1]
                    + "  p3: " + (slotLoaded[2] ? "● " : "○ ") + slotColors[2]);
            telemetryM.update(telemetry);
            // telemetry.update() is handled by telemetryM.update(telemetry)
        }
    }
}
