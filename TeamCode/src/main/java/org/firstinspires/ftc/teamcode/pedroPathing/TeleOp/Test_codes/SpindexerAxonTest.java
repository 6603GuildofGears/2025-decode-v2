package org.firstinspires.ftc.teamcode.pedroPathing.TeleOp.Test_codes;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.CRServo;

import org.firstinspires.ftc.teamcode.pedroPathing.Pipelines.RTPAxon;

/**
 * Test OpMode for RTPAxon spindexer.
 *
 * Gamepad1 controls:
 *   D-pad Up    → go to P1 (0°)
 *   D-pad Right → go to P2 (120°)
 *   D-pad Down  → go to P3 (240°)
 *   A           → nudge target +15°
 *   B           → nudge target -15°
 *   X           → reset to 0°
 *   Y           → toggle RTP mode on/off (manual power with left stick Y)
 *
 *   Right bumper → increase kP by 0.0005
 *   Left bumper  → decrease kP by 0.0005
 *   Right trigger → increase kD by 0.0005
 *   Left trigger  → decrease kD by 0.0005
 *
 * ─────────────────────────────────────────────
 *  HARDWARE CONFIG — update these names to match your wiring:
 *    CR Servo:       "spindexerCR"
 *    Analog Encoder: "spindexerEncoder"
 * ─────────────────────────────────────────────
 */
@TeleOp(name = "Spindexer RTPAxon Test", group = "Test")
public class SpindexerAxonTest extends LinearOpMode {

    // ── Slot positions in degrees ──
    // These correspond to your current P1/P2/P3 positions
    // Tune these to match where the slots actually line up
    private static final double P1_DEG = 0;
    private static final double P2_DEG = 120;
    private static final double P3_DEG = 240;

    // Button edge-detection
    private boolean prevY = false;
    private boolean prevA = false;
    private boolean prevB = false;
    private boolean prevX = false;
    private boolean prevDU = false;
    private boolean prevDR = false;
    private boolean prevDD = false;
    private boolean prevRB = false;
    private boolean prevLB = false;

    @Override
    public void runOpMode() {

        // ── Hardware ──
        CRServo crServo = hardwareMap.crservo.get("spindexerCR");
        AnalogInput encoder = hardwareMap.get(AnalogInput.class, "spindexerEncoder");

        // ── Create RTPAxon (no direction flip — motor and encoder already agree) ──
        RTPAxon spindexer = new RTPAxon(crServo, encoder);
        spindexer.setMaxPower(0.3);
        spindexer.setKP(0.008);    // moderate P — now safe with filtered encoder
        spindexer.setKI(0.0);      // no integral for now
        spindexer.setKD(0.004);    // moderate damping — filtered, won't blow up

        telemetry.addData("Status", "RTPAxon initialized");
        telemetry.addData("Start Angle", String.format("%.1f°", spindexer.STARTPOS));
        telemetry.addData(">", "Press START");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {

            // ── Edge detection ──
            boolean y  = gamepad1.y;
            boolean a  = gamepad1.a;
            boolean b  = gamepad1.b;
            boolean x  = gamepad1.x;
            boolean du = gamepad1.dpad_up;
            boolean dr = gamepad1.dpad_right;
            boolean dd = gamepad1.dpad_down;

            // ── Slot presets (D-pad) ──
            if (du && !prevDU) spindexer.setTargetRotation(P1_DEG);
            if (dr && !prevDR) spindexer.setTargetRotation(P2_DEG);
            if (dd && !prevDD) spindexer.setTargetRotation(P3_DEG);

            // ── Fine adjust ──
            if (a && !prevA) spindexer.changeTargetRotation(15);
            if (b && !prevB) spindexer.changeTargetRotation(-15);

            // ── Reset ──
            if (x && !prevX) {
                spindexer.setTargetRotation(0);
                spindexer.forceResetTotalRotation();
            }

            // ── Toggle RTP / Manual ──
            if (y && !prevY) {
                spindexer.setRtp(!spindexer.getRtp());
            }

            // ── Live PID tuning (bumpers & triggers) ──
            boolean rb = gamepad1.right_bumper;
            boolean lb = gamepad1.left_bumper;
            if (rb && !prevRB) spindexer.setKP(spindexer.getKP() + 0.0005);
            if (lb && !prevLB) spindexer.setKP(Math.max(0, spindexer.getKP() - 0.0005));
            if (gamepad1.right_trigger > 0.5) spindexer.setKD(spindexer.getKD() + 0.0001);
            if (gamepad1.left_trigger  > 0.5) spindexer.setKD(Math.max(0, spindexer.getKD() - 0.0001));

            // ── Manual drive when RTP is off ──
            if (!spindexer.getRtp()) {
                double manual = -gamepad1.left_stick_y * spindexer.getMaxPower();
                spindexer.setPower(manual);
            }

            // ── Update PID loop ──
            spindexer.update();

            // ── Telemetry ──
            telemetry.addData("=== MODE ===", spindexer.getRtp() ? "RUN-TO-POSITION" : "MANUAL (left stick Y)");
            telemetry.addData("kP", String.format("%.4f", spindexer.getKP()));
            telemetry.addData("kD", String.format("%.4f", spindexer.getKD()));
            telemetry.addData("Max Power", String.format("%.3f", spindexer.getMaxPower()));
            telemetry.addData("At Target?", spindexer.isAtTarget() ? "YES ✓" : "NO");
            telemetry.addLine("");
            telemetry.addLine(spindexer.log());
            telemetry.addLine("");
            telemetry.addData("=== CONTROLS ===", "");
            telemetry.addData("D-Up/Right/Down", "P1(0°) / P2(120°) / P3(240°)");
            telemetry.addData("A / B", "Nudge ±15°");
            telemetry.addData("X", "Reset to 0°");
            telemetry.addData("Y", "Toggle RTP / Manual");
            telemetry.addData("RB / LB", "kP ± 0.0005");
            telemetry.addData("RT / LT", "kD ± 0.0001");
            telemetry.update();

            // ── Save edge states ──
            prevY = y; prevA = a; prevB = b; prevX = x;
            prevDU = du; prevDR = dr; prevDD = dd;
            prevRB = rb; prevLB = lb;
        }

        spindexer.setPower(0);
    }
}
