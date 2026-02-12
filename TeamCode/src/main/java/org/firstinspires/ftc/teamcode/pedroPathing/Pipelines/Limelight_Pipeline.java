package org.firstinspires.ftc.teamcode.pedroPathing.Pipelines;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;

public class Limelight_Pipeline {

    public static Limelight3A limelight;

    // Cached result — call pollOnce() at the TOP of every loop iteration
    // so every getter in that loop sees the SAME frame.
    // If pollOnce() is never called (old OpModes), getters poll live.
    private static LLResult cachedResult = null;
    private static boolean  cacheValid   = false;

    /**
     * Initialize the Limelight in AprilTag mode (pipeline 0).
     * Always re-acquires from hardwareMap so stale refs don't survive
     * across OpMode restarts.
     */
    public static void initLimelight(OpMode opMode) {
        try {
            // Always grab a fresh reference (static field survives between OpModes)
            limelight = opMode.hardwareMap.get(Limelight3A.class, "limelight");

            // Stop first to reset
            limelight.stop();

            // Set to AprilTag pipeline (pipeline 0)
            limelight.pipelineSwitch(0);

            // Set polling mode to enable camera streaming
            limelight.setPollRateHz(100); // 100Hz polling for smooth tracking

            // Start the Limelight
            limelight.start();

            // Give it a moment to initialize
            try { Thread.sleep(100); } catch (InterruptedException e) {}

            // Force an initial poll
            LLResult testResult = limelight.getLatestResult();

            opMode.telemetry.addData("Limelight", "Initialized - AprilTag Mode");
            opMode.telemetry.addData("LL Streaming", "Enabled at 100Hz");
            opMode.telemetry.addData("LL Status", testResult != null ? "Connected" : "No Data");
            opMode.telemetry.update();
        } catch (Exception e) {
            opMode.telemetry.addData("LL ERROR", e.getMessage());
            opMode.telemetry.update();
        }
    }

    /**
     * Poll the Limelight ONCE per loop and cache the result.
     * Call this at the TOP of every loop() before any has/get methods.
     * When used, all getters in that loop iteration see the SAME frame.
     */
    public static void pollOnce() {
        if (limelight != null) {
            cachedResult = limelight.getLatestResult();
            cacheValid   = true;
        } else {
            cachedResult = null;
            cacheValid   = false;
        }
    }

    /**
     * Internal helper — returns the cached result if pollOnce() was called,
     * otherwise polls the hardware live (backward-compatible for old OpModes).
     */
    private static LLResult getResult() {
        if (cacheValid) return cachedResult;
        // Fallback: poll live for OpModes that never call pollOnce()
        return (limelight != null) ? limelight.getLatestResult() : null;
    }

    /**
     * Get the latest result from the Limelight.
     */
    public static LLResult getLatestResult() {
        return getResult();
    }

    /**
     * Check if a target is detected
     */
    public static boolean hasTarget() {
        LLResult r = getResult();
        return r != null && r.isValid();
    }

    /**
     * Get the horizontal offset (tx) of the first target
     */
    public static double getTargetX() {
        LLResult r = getResult();
        if (r != null && r.isValid()
                && r.getFiducialResults() != null
                && !r.getFiducialResults().isEmpty()) {
            return r.getFiducialResults().get(0).getTargetXDegrees();
        }
        return 0.0;
    }

    /**
     * Get the vertical offset (ty) of the first target
     */
    public static double getTargetY() {
        LLResult r = getResult();
        if (r != null && r.isValid()
                && r.getFiducialResults() != null
                && !r.getFiducialResults().isEmpty()) {
            return r.getFiducialResults().get(0).getTargetYDegrees();
        }
        return 0.0;
    }

    /**
     * Check if blue goal (ID 20) is detected
     */
    public static boolean hasBlueGoal() {
        LLResult r = getResult();
        if (r != null && r.isValid() && r.getFiducialResults() != null) {
            for (int i = 0; i < r.getFiducialResults().size(); i++) {
                if ((int) r.getFiducialResults().get(i).getFiducialId() == 20) {
                    return true;
                }
            }
        }
        return false;
    }

    /**
     * Check if red goal (ID 24) is detected
     */
    public static boolean hasRedGoal() {
        LLResult r = getResult();
        if (r != null && r.isValid() && r.getFiducialResults() != null) {
            for (int i = 0; i < r.getFiducialResults().size(); i++) {
                if ((int) r.getFiducialResults().get(i).getFiducialId() == 24) {
                    return true;
                }
            }
        }
        return false;
    }

    /**
     * Get horizontal offset (tx) for blue goal (ID 20)
     */
    public static double getBlueGoalX() {
        LLResult r = getResult();
        if (r != null && r.isValid() && r.getFiducialResults() != null) {
            for (int i = 0; i < r.getFiducialResults().size(); i++) {
                if ((int) r.getFiducialResults().get(i).getFiducialId() == 20) {
                    return r.getFiducialResults().get(i).getTargetXDegrees();
                }
            }
        }
        return 0.0;
    }

    /**
     * Get vertical offset (ty) for blue goal (ID 20)
     */
    public static double getBlueGoalY() {
        LLResult r = getResult();
        if (r != null && r.isValid() && r.getFiducialResults() != null) {
            for (int i = 0; i < r.getFiducialResults().size(); i++) {
                if ((int) r.getFiducialResults().get(i).getFiducialId() == 20) {
                    return r.getFiducialResults().get(i).getTargetYDegrees();
                }
            }
        }
        return 0.0;
    }

    /**
     * Get horizontal offset (tx) for red goal (ID 24)
     */
    public static double getRedGoalX() {
        LLResult r = getResult();
        if (r != null && r.isValid() && r.getFiducialResults() != null) {
            for (int i = 0; i < r.getFiducialResults().size(); i++) {
                if ((int) r.getFiducialResults().get(i).getFiducialId() == 24) {
                    return r.getFiducialResults().get(i).getTargetXDegrees();
                }
            }
        }
        return 0.0;
    }

    /**
     * Get vertical offset (ty) for red goal (ID 24)
     */
    public static double getRedGoalY() {
        LLResult r = getResult();
        if (r != null && r.isValid() && r.getFiducialResults() != null) {
            for (int i = 0; i < r.getFiducialResults().size(); i++) {
                if ((int) r.getFiducialResults().get(i).getFiducialId() == 24) {
                    return r.getFiducialResults().get(i).getTargetYDegrees();
                }
            }
        }
        return 0.0;
    }

    /**
     * Display telemetry for detected AprilTags.
     * If using pollOnce(), call it before this.
     */
    public static void displayTelemetry(OpMode opMode) {
        LLResult r = getResult();

        opMode.telemetry.addData("LL Connected", r != null);

        if (r != null && r.isValid() && r.getFiducialResults() != null) {
            opMode.telemetry.addData("LL Targets", r.getFiducialResults().size());

            for (int i = 0; i < r.getFiducialResults().size(); i++) {
                int tagId = (int) r.getFiducialResults().get(i).getFiducialId();
                double tx = r.getFiducialResults().get(i).getTargetXDegrees();

                String goalName = "";
                if (tagId == 20) goalName = " (BLUE GOAL)";
                else if (tagId == 24) goalName = " (RED GOAL)";

                opMode.telemetry.addData("Tag " + tagId + goalName, String.format("X: %.2f°", tx));
            }
        } else {
            opMode.telemetry.addData("LL Status", "No targets");
        }
    }
}
