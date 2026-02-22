package org.firstinspires.ftc.teamcode.pedroPathing.TeleOp.Test_codes;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.CRServo;

import org.firstinspires.ftc.teamcode.pedroPathing.Pipelines.RTPAxon;

/**
 * Spindexer RTPAxon Test
 *
 * 3 slots spaced 120° apart. Two base angles define all 6 positions:
 *   Intake: 100°, 220°, 340°
 *   Shoot:  165°, 285°, 45°
 *
 * Y toggles between INTAKE and SHOOT mode.
 * D-pad Up/Right/Down goes to P1/P2/P3 of the current mode.
 *
 * ─────────────────────────────────────────────
 *  HARDWARE:
 *    CR Servo:       "spindexerCR"
 *    Analog Encoder: "spindexerEncoder"
 * ─────────────────────────────────────────────
 */
@TeleOp(name = "Spindexer RTPAxon Test", group = "Test")
public class SpindexerAxonTest extends LinearOpMode {

    // Slot spacing — 3 evenly spaced slots
    private static final double SLOT_SPACING = 120.0;

    // Tuned base angles
    private static final double INTAKE_BASE = 100;
    private static final double SHOOT_BASE  = 165;

    // Mode: true = intake positions, false = shoot positions
    private boolean intakeMode = true;

    // Button edge-detection
    private boolean prevY = false;
    private boolean prevDU = false;
    private boolean prevDR = false;
    private boolean prevDD = false;
    private boolean prevDL = false;
    private boolean prevA = false;
    private boolean prevLB = false;
    private boolean prevRB = false;

    // Tuning parameter selector
    // 0 = kP, 1 = kD, 2 = maxPower, 3 = minPower
    private int tuneParam = 0;
    private static final String[] PARAM_NAMES = {"kP", "kD", "maxPwr", "minPwr"};
    private static final double[] PARAM_STEPS = {0.0001, 0.00001, 0.05, 0.005};

    /** Wrap angle to 0–360 range */
    private double wrap360(double deg) {
        deg %= 360;
        if (deg < 0) deg += 360;
        return deg;
    }

    /** Adjust the selected tuning parameter by delta */
    private void adjustParam(RTPAxon axon, int param, double delta) {
        switch (param) {
            case 0: axon.setKP(Math.max(0, axon.getKP() + delta)); break;
            case 1: axon.setKD(Math.max(0, axon.getKD() + delta)); break;
            case 2: axon.setMaxPower(Math.max(0.05, axon.getMaxPower() + delta)); break;
            case 3: axon.setMinPower(Math.max(0, axon.getMinPower() + delta)); break;
        }
    }

    @Override
    public void runOpMode() {

        // ── Hardware ──
        CRServo crServo = hardwareMap.crservo.get("spindexerCR");
        AnalogInput encoder = hardwareMap.get(AnalogInput.class, "spindexerEncoder");

        // ── Create RTPAxon ──
        RTPAxon spindexer = new RTPAxon(crServo, encoder);
        spindexer.setMaxPower(0.4);
        spindexer.setMinPower(0.05);
        spindexer.setKP(0.003);
        spindexer.setKI(0.0);
        spindexer.setKD(0.00012);

        telemetry.addData("Status", "RTPAxon initialized");
        telemetry.addData("Start Angle", String.format("%.1f°", spindexer.STARTPOS));
        telemetry.addData(">", "Press START — Y toggles INTAKE/SHOOT");
        telemetry.update();

        waitForStart();

        // Derive all 6 positions
        double intakeP1 = wrap360(INTAKE_BASE);
        double intakeP2 = wrap360(INTAKE_BASE + SLOT_SPACING);
        double intakeP3 = wrap360(INTAKE_BASE + SLOT_SPACING * 2);
        double shootP1  = wrap360(SHOOT_BASE);
        double shootP2  = wrap360(SHOOT_BASE + SLOT_SPACING);
        double shootP3  = wrap360(SHOOT_BASE + SLOT_SPACING * 2);

        while (opModeIsActive()) {

            // ── Read buttons ──
            boolean y  = gamepad1.y;
            boolean du = gamepad1.dpad_up;
            boolean dr = gamepad1.dpad_right;
            boolean dd = gamepad1.dpad_down;
            boolean dl = gamepad1.dpad_left;
            boolean a  = gamepad1.a;
            boolean lb = gamepad1.left_bumper;
            boolean rb = gamepad1.right_bumper;

            // ── Toggle INTAKE / SHOOT mode ──
            if (y && !prevY) intakeMode = !intakeMode;

            // ── A cycles tuning parameter ──
            if (a && !prevA) tuneParam = (tuneParam + 1) % PARAM_NAMES.length;

            // ── Bumpers adjust selected parameter ──
            if (lb && !prevLB) {
                adjustParam(spindexer, tuneParam, -PARAM_STEPS[tuneParam]);
            }
            if (rb && !prevRB) {
                adjustParam(spindexer, tuneParam, +PARAM_STEPS[tuneParam]);
            }

            // ── Update controller FIRST so telemetry shows current-cycle values ──
            spindexer.update();

            // ── D-pad selects P1/P2/P3 of current mode ──
            if (intakeMode) {
                if (du && !prevDU) spindexer.setTargetRotation(intakeP1);
                if (dr && !prevDR) spindexer.setTargetRotation(intakeP2);
                if (dd && !prevDD) spindexer.setTargetRotation(intakeP3);
            } else {
                if (du && !prevDU) spindexer.setTargetRotation(shootP1);
                if (dr && !prevDR) spindexer.setTargetRotation(shootP2);
                if (dd && !prevDD) spindexer.setTargetRotation(shootP3);
            }

            // ── Telemetry ──
            telemetry.addData("MODE", intakeMode ? ">>> INTAKE <<<" : ">>> SHOOT <<<");
            telemetry.addLine(spindexer.log());
            telemetry.addData("At Target?", spindexer.isAtTarget() ? "YES" : "NO");
            telemetry.addLine("─── POSITIONS ───");
            telemetry.addData("Intake", String.format("P1=%.0f°  P2=%.0f°  P3=%.0f°", intakeP1, intakeP2, intakeP3));
            telemetry.addData("Shoot ", String.format("P1=%.0f°  P2=%.0f°  P3=%.0f°", shootP1, shootP2, shootP3));
            telemetry.addLine("─── CONTROLS ───");
            telemetry.addData("Y", "toggle INTAKE/SHOOT");
            telemetry.addData("D-Up/Right/Down", "P1 / P2 / P3");
            telemetry.addLine("─── TUNING ───");
            telemetry.addData("A", "cycle param");
            telemetry.addData("LB / RB", String.format("adjust (%s)", PARAM_NAMES[tuneParam]));
            telemetry.addData("► " + PARAM_NAMES[0], String.format("%s%.4f  step=%.4f",
                    tuneParam == 0 ? ">> " : "   ", spindexer.getKP(), PARAM_STEPS[0]));
            telemetry.addData("► " + PARAM_NAMES[1], String.format("%s%.5f  step=%.5f",
                    tuneParam == 1 ? ">> " : "   ", spindexer.getKD(), PARAM_STEPS[1]));
            telemetry.addData("► " + PARAM_NAMES[2], String.format("%s%.3f  step=%.3f",
                    tuneParam == 2 ? ">> " : "   ", spindexer.getMaxPower(), PARAM_STEPS[2]));
            telemetry.addData("► " + PARAM_NAMES[3], String.format("%s%.4f  step=%.4f",
                    tuneParam == 3 ? ">> " : "   ", spindexer.getMinPower(), PARAM_STEPS[3]));

            // ── Telemetry flush ──
            telemetry.update();

            // ── Save edge states ──
            prevY = y;
            prevDU = du; prevDR = dr; prevDD = dd; prevDL = dl;
            prevA = a; prevLB = lb; prevRB = rb;
        }

        spindexer.setPower(0);
    }
}
