package org.firstinspires.ftc.teamcode.pedroPathing.TeleOp.Test_codes;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

import org.firstinspires.ftc.teamcode.pedroPathing.Pipelines.Limelight_Pipeline;
import org.firstinspires.ftc.teamcode.pedroPathing.Pipelines.Motif;

/**
 * Simple test OpMode — point the Limelight at the obelisk AprilTag
 * and it tells you the ball shoot order for TeleOp.
 *
 * Tag 21 → GREEN, PURPLE, PURPLE
 * Tag 22 → PURPLE, GREEN, PURPLE
 * Tag 23 → PURPLE, PURPLE, GREEN
 */
@Disabled
@TeleOp(name = "Motif Test", group = "Test")
public class MotifTest extends LinearOpMode {

    @Override
    public void runOpMode() {

        // Initialize Limelight (AprilTag pipeline)
        Limelight_Pipeline.initLimelight(this);

        telemetry.addData("Status", "Ready — point Limelight at obelisk");
        telemetry.addData(">", "Press START or just watch init");
        telemetry.update();

        int lockedTagId = -1;
        String[] lockedOrder = null;

        // ── INIT LOOP: scan while waiting for start ──
        while (!isStarted() && !isStopRequested()) {
            Limelight_Pipeline.pollOnce();
            int tagId = Limelight_Pipeline.getObeliskTagId();

            if (tagId != -1) {
                lockedTagId = tagId;
                lockedOrder = Motif.getShootOrder(tagId);
            }

            showMotif(lockedTagId, lockedOrder, tagId != -1);
            telemetry.update();
            sleep(50);
        }

        waitForStart();

        // ── MAIN LOOP: keep scanning live ──
        while (opModeIsActive()) {
            Limelight_Pipeline.pollOnce();
            int tagId = Limelight_Pipeline.getObeliskTagId();

            if (tagId != -1) {
                lockedTagId = tagId;
                lockedOrder = Motif.getShootOrder(tagId);
            }

            showMotif(lockedTagId, lockedOrder, tagId != -1);
            telemetry.update();
            sleep(50);
        }
    }

    private void showMotif(int tagId, String[] order, boolean liveDetection) {
        telemetry.addData("── MOTIF DETECTOR ──", "");

        if (tagId == -1) {
            telemetry.addData("Status", "No obelisk tag found");
            telemetry.addData("Looking for", "Tag 21, 22, or 23");
            telemetry.addData("Tip", "Point Limelight at the obelisk");
        } else {
            telemetry.addData("Motif", Motif.getMotifName(tagId));
            telemetry.addData("Live", liveDetection ? "YES (seeing tag now)" : "no (using last seen)");
            telemetry.addLine("");
            telemetry.addData("=== BALL ORDER FOR TELEOP ===", "");
            telemetry.addData("  1st ball to shoot", order[0]);
            telemetry.addData("  2nd ball to shoot", order[1]);
            telemetry.addData("  3rd ball to shoot", order[2]);
            telemetry.addLine("");
            telemetry.addData("Sequence", order[0] + " → " + order[1] + " → " + order[2]);
        }
    }
}
