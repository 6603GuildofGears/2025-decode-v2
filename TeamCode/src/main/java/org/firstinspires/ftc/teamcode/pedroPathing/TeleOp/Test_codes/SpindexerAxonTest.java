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
        spindexer.setMaxPower(1.0);    // full power allowed (ramps down near target via P)
        spindexer.setKP(0.0025);       // sweet spot — no overshoot
        spindexer.setKI(0.0);          // no integral for now
        spindexer.setKD(0.0);          // no D for now — tune P first

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
            if (rb && !prevRB) spindexer.setKP(spindexer.getKP() + 0.0001);
            if (lb && !prevLB) spindexer.setKP(Math.max(0, spindexer.getKP() - 0.0001));
            // Triggers: adjust maxPower ± 0.01
            if (gamepad1.right_trigger > 0.5) spindexer.setMaxPower(spindexer.getMaxPower() + 0.005);
            if (gamepad1.left_trigger  > 0.5) spindexer.setMaxPower(Math.max(0.02, spindexer.getMaxPower() - 0.005));

            // ── Manual drive when RTP is off ──
            if (!spindexer.getRtp()) {
                double manual = -gamepad1.left_stick_y * spindexer.getMaxPower();
                spindexer.setPower(manual);
            }

            // ── Update PID loop ──
            spindexer.update();

            // ── Telemetry ──
            telemetry.addData("MODE", spindexer.getRtp() ? "RTP" : "MANUAL");
            telemetry.addData("At Target?", spindexer.isAtTarget() ? "YES" : "NO");
            telemetry.addLine(spindexer.log());
            telemetry.addData("maxPower", String.format("%.3f", spindexer.getMaxPower()));
            telemetry.addLine("---");
            telemetry.addData("Dpad", "P1/P2/P3 | A/B nudge");
            telemetry.addData("X:reset Y:mode", "RB/LB:kP RT/LT:maxPwr");
            telemetry.update();

            // ── Save edge states ──
            prevY = y; prevA = a; prevB = b; prevX = x;
            prevDU = du; prevDR = dr; prevDD = dd;
            prevRB = rb; prevLB = lb;
        }

        spindexer.setPower(0);
    }
}
