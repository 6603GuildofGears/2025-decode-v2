package org.firstinspires.ftc.teamcode.pedroPathing.TeleOp.Test_codes;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.pedroPathing.Pipelines.TurretSubsystem;

import java.io.FileWriter;
import java.io.IOException;
import java.io.PrintWriter;
import java.util.ArrayList;

/**
 * ========================================================================
 *  TURRET PID AUTO-TUNER
 * ========================================================================
 *
 * Automatic PID gain tuner for the turret subsystem using step-response
 * analysis. Runs as a standalone OpMode.
 *
 * ========================== HOW TO USE ==========================
 *
 * STEP 1: Place the robot on a flat surface with the turret free to rotate.
 * STEP 2: Select "Turret PID Tuner" from the Driver Station.
 * STEP 3: Press INIT — the turret will home to the magnetic limit switch.
 * STEP 4: Press PLAY — the tuner is ready.
 * STEP 5: Press A to run an auto-tune iteration.
 *         The turret will step from 165° → 50° and back, measuring response.
 * STEP 6: Watch telemetry for results. The tuner adjusts gains automatically.
 * STEP 7: Repeat (press A again) until all metrics show "GOOD" or 10 iterations.
 * STEP 8: Press B to accept the final gains — they'll be logged to a file.
 * STEP 9: Press X at any time to reset gains and start over.
 * STEP 10: Press Y for EMERGENCY STOP at any time.
 *
 * STEP 11: Copy the final P, I, D values from telemetry into your OpMode
 *          or TurretSubsystem defaults.
 *
 * ========================== ALGORITHM ==========================
 *
 * Uses a simplified Ziegler-Nichols approach with step response analysis:
 *
 * 1. Start with conservative gains (P=0.01, I=0, D=0)
 * 2. Command a large step move (165° → 50°)
 * 3. Sample position every ~20ms for 3 seconds
 * 4. Analyze:
 *    - Rise time:       Time to reach 90% of target
 *    - Overshoot:       Max deviation past target (degrees)
 *    - Settling time:   Time to stay within ±2° of target
 *    - Steady-state error: Final position error
 *    - Oscillation:     Zero-crossings of error signal
 * 5. Adjust gains:
 *    - Overshoot >10%:  Decrease P by 20%, increase D by 50%
 *    - No overshoot, slow rise: Increase P by 30%
 *    - Steady-state error >3°: Add/increase I term
 *    - Oscillating (>3 zero-crossings): Decrease P by 30%, increase D by 100%
 * 6. Repeat up to 10 iterations or until all criteria met:
 *    - Overshoot < 5°
 *    - Settling time < 1.5 seconds
 *    - Steady-state error < 2°
 *
 * ========================== SAFETY ==========================
 *
 * - Aborts if turret position exceeds 0-330° range
 * - Aborts if motor stalls (no encoder change for >500ms)
 * - Emergency stop: gamepad1.y at any time
 * - Max power capped at 0.7 during tuning
 *
 * ========================== DATA LOGGING ==========================
 *
 * Each tuning session is saved to:
 *   /sdcard/FIRST/turret_pid_tuning.csv
 * Format: iteration, P, I, D, overshoot, settlingTime, ssError, oscillations
 */
@TeleOp(name = "Turret PID Tuner", group = "Test")
public class TurretPIDTuner extends LinearOpMode {

    // ========================== TUNING PARAMETERS ==========================

    /** Starting position for step tests (center of range). */
    private static final double START_ANGLE = 165.0;

    /** Target position for step tests (large step to measure response). */
    private static final double TEST_ANGLE = 50.0;

    /** How long to record data after commanding the step (seconds). */
    private static final double RECORD_DURATION_SEC = 3.0;

    /** Sampling interval in milliseconds. */
    private static final long SAMPLE_INTERVAL_MS = 20;

    /** Maximum tuning iterations before giving up. */
    private static final int MAX_ITERATIONS = 10;

    /** Step size in degrees for the test move. */
    private static final double STEP_SIZE = Math.abs(START_ANGLE - TEST_ANGLE); // 115°

    // ========================== ACCEPTANCE CRITERIA ==========================
    // When ALL of these are met, tuning is considered complete.

    private static final double MAX_ACCEPTABLE_OVERSHOOT_DEG = 5.0;
    private static final double MAX_ACCEPTABLE_SETTLING_SEC = 1.5;
    private static final double MAX_ACCEPTABLE_SS_ERROR_DEG = 2.0;
    private static final int    MAX_ACCEPTABLE_OSCILLATIONS = 3;

    // ========================== INITIAL GAINS ==========================
    // Very conservative starting point. The tuner will increase from here.

    private double tuneP = 0.010;
    private double tuneI = 0.0000;
    private double tuneD = 0.0000;

    // ========================== STATE ==========================

    private TurretSubsystem turret;
    private int iteration = 0;

    // Last test results
    private double lastRiseTime = -1;
    private double lastOvershoot = -1;
    private double lastSettlingTime = -1;
    private double lastSSError = -1;
    private int    lastOscillations = -1;
    private boolean tuningComplete = false;
    private boolean testInProgress = false;

    // Data log for the current test
    private ArrayList<DataPoint> testData = new ArrayList<>();

    /** Single data sample: time + position. */
    private static class DataPoint {
        double timeSec;
        double angleDeg;

        DataPoint(double t, double a) {
            timeSec = t;
            angleDeg = a;
        }
    }

    // ========================== MAIN ==========================

    @Override
    public void runOpMode() throws InterruptedException {

        // --- Hardware init ---
        DcMotorEx turretMotor = hardwareMap.get(DcMotorEx.class, "turret");
        turretMotor.setDirection(com.qualcomm.robotcore.hardware.DcMotor.Direction.FORWARD);

        TouchSensor mag = hardwareMap.get(TouchSensor.class, "mag");

        turret = new TurretSubsystem(turretMotor, mag);
        turret.setMaxPower(0.7); // Conservative for tuning

        // --- Show diagnostic info during init ---
        // Don't auto-home during init — let the user see sensor state first
        while (!isStarted() && !isStopRequested()) {
            telemetry.addLine("=== TURRET PID AUTO-TUNER ===");
            telemetry.addLine("");
            telemetry.addData("Mag Sensor", mag.isPressed() ? "PRESSED" : "NOT PRESSED");
            telemetry.addData("Encoder", turretMotor.getCurrentPosition());
            telemetry.addLine("");
            telemetry.addLine("Press PLAY to start.");
            telemetry.addLine("Turret will home first, then you can tune.");
            telemetry.update();
            sleep(50);
        }

        if (isStopRequested()) return;

        // --- Attempt homing after PLAY ---
        telemetry.clearAll();
        telemetry.addLine("Homing turret...");
        telemetry.addLine("Motor should rotate toward the magnet.");
        telemetry.update();

        boolean homeOk = turret.home();

        if (!homeOk) {
            // Homing failed — give user the option to bypass
            telemetry.clearAll();
            telemetry.addLine("!! HOMING FAILED !!");
            telemetry.addLine("");
            telemetry.addLine("The turret didn't find the magnet in 5 seconds.");
            telemetry.addLine("Possible causes:");
            telemetry.addLine("  - Motor rotated the WRONG direction");
            telemetry.addLine("  - Magnet sensor wiring issue");
            telemetry.addLine("  - Turret was physically stuck");
            telemetry.addLine("");
            telemetry.addData("Mag Sensor RIGHT NOW", mag.isPressed() ? "PRESSED" : "NOT PRESSED");
            telemetry.addData("Encoder", turretMotor.getCurrentPosition());
            telemetry.addLine("");
            telemetry.addLine("[A] Skip homing — use current position as 0°");
            telemetry.addLine("[B] Try motor test — spin at 0.3 power for 1 sec");
            telemetry.addLine("[X] Retry homing");
            telemetry.update();

            // Wait for user decision
            while (opModeIsActive()) {
                if (gamepad1.a) {
                    // Bypass homing: zero encoder at current position
                    turret.forceHomed();
                    telemetry.clearAll();
                    telemetry.addLine("Homing bypassed — current position = 0°");
                    telemetry.update();
                    sleep(1000);
                    break;
                }
                if (gamepad1.b) {
                    // Quick motor test — spin at 0.3 power for 1 second
                    telemetry.clearAll();
                    telemetry.addLine("Motor test: spinning at 0.3 power...");
                    telemetry.update();
                    int startTicks = turretMotor.getCurrentPosition();
                    turretMotor.setPower(0.3);
                    sleep(1000);
                    turretMotor.setPower(0);
                    int endTicks = turretMotor.getCurrentPosition();
                    telemetry.clearAll();
                    telemetry.addLine("Motor test complete.");
                    telemetry.addData("Start ticks", startTicks);
                    telemetry.addData("End ticks", endTicks);
                    telemetry.addData("Delta", endTicks - startTicks);
                    telemetry.addData("Mag NOW", mag.isPressed() ? "PRESSED" : "NOT PRESSED");
                    telemetry.addLine("");
                    if (Math.abs(endTicks - startTicks) < 5) {
                        telemetry.addLine("!! Motor did NOT move. Check wiring/config.");
                    } else {
                        telemetry.addLine("Motor moved! Direction might be wrong for homing.");
                        telemetry.addLine("Press A to use current position as 0° and continue.");
                    }
                    telemetry.addLine("");
                    telemetry.addLine("[A] Skip homing  |  [X] Retry homing");
                    telemetry.update();
                }
                if (gamepad1.x) {
                    // Retry homing
                    telemetry.clearAll();
                    telemetry.addLine("Retrying homing...");
                    telemetry.update();
                    homeOk = turret.home();
                    if (homeOk) {
                        telemetry.clearAll();
                        telemetry.addLine("Homing succeeded!");
                        telemetry.update();
                        sleep(1000);
                        break;
                    } else {
                        telemetry.clearAll();
                        telemetry.addLine("Homing failed again.");
                        telemetry.addData("Mag NOW", mag.isPressed() ? "PRESSED" : "NOT PRESSED");
                        telemetry.addLine("[A] Skip  |  [B] Motor test  |  [X] Retry");
                        telemetry.update();
                    }
                }
                sleep(50);
            }
        } else {
            telemetry.clearAll();
            telemetry.addLine("Homed successfully! Encoder zeroed at magnet.");
            telemetry.update();
            sleep(500);
        }

        if (!opModeIsActive()) return;

        // Move to start position
        turret.setPIDGains(tuneP, tuneI, tuneD);
        turret.setTargetAngle(START_ANGLE);

        ElapsedTime settleTimer = new ElapsedTime();
        settleTimer.reset();

        // Wait for turret to reach start position (up to 4 seconds)
        while (opModeIsActive() && settleTimer.seconds() < 4.0) {
            turret.update();
            telemetry.addLine("Moving to start position (165°)...");
            telemetry.addData("Current", "%.1f°", turret.getCurrentAngle());
            telemetry.update();
            if (turret.isAtTarget(3.0)) break;
            sleep(20);
        }

        // === Main tuner loop ===
        while (opModeIsActive()) {

            // --- Emergency stop ---
            if (gamepad1.y) {
                turret.emergencyStop();
                telemetry.clearAll();
                telemetry.addLine("!! EMERGENCY STOP !!");
                telemetry.addLine("Press X to reset, or stop OpMode.");
                telemetry.update();
                while (opModeIsActive() && !gamepad1.x) {
                    sleep(50);
                }
                if (gamepad1.x) {
                    turret.clearEmergencyStop();
                    resetTuner();
                }
                continue;
            }

            // --- Run test (A button) ---
            if (gamepad1.a && !testInProgress && !tuningComplete) {
                runTestIteration();
                continue;
            }

            // --- Accept gains (B button) ---
            if (gamepad1.b && !testInProgress) {
                saveResults();
                telemetry.clearAll();
                telemetry.addLine("=== GAINS ACCEPTED ===");
                telemetry.addLine("");
                telemetry.addData("Final P", "%.6f", tuneP);
                telemetry.addData("Final I", "%.6f", tuneI);
                telemetry.addData("Final D", "%.6f", tuneD);
                telemetry.addLine("");
                telemetry.addLine("Values saved to:");
                telemetry.addLine("/sdcard/FIRST/turret_pid_tuning.csv");
                telemetry.addLine("");
                telemetry.addLine("Copy these into TurretSubsystem.java!");
                telemetry.update();
                while (opModeIsActive() && !gamepad1.x) {
                    sleep(50);
                }
                resetTuner();
                continue;
            }

            // --- Reset (X button) ---
            if (gamepad1.x && !testInProgress) {
                resetTuner();
            }

            // --- Keep turret at start position ---
            turret.update();

            // --- Display UI ---
            displayTelemetry();

            sleep(20);
        }

        // Cleanup
        turret.emergencyStop();
    }

    // ========================== TEST ITERATION ==========================

    /**
     * Execute one complete step-response test:
     * 1. Ensure turret is at start position
     * 2. Command step to test angle
     * 3. Record position data for RECORD_DURATION_SEC
     * 4. Analyze response
     * 5. Adjust gains based on analysis
     * 6. Return to start position
     */
    private void runTestIteration() {
        testInProgress = true;
        iteration++;

        // Apply current gains
        turret.setPIDGains(tuneP, tuneI, tuneD);

        // --- Phase 1: Ensure at start position ---
        turret.setTargetAngle(START_ANGLE);
        ElapsedTime phaseTimer = new ElapsedTime();
        phaseTimer.reset();

        while (opModeIsActive() && phaseTimer.seconds() < 3.0) {
            turret.update();
            telemetry.clearAll();
            telemetry.addLine("=== PREPARING TEST " + iteration + "/" + MAX_ITERATIONS + " ===");
            telemetry.addData("Moving to start", "%.1f° → %.1f°", turret.getCurrentAngle(), START_ANGLE);
            telemetry.addData("Gains", "P=%.5f  I=%.6f  D=%.5f", tuneP, tuneI, tuneD);
            telemetry.update();
            if (turret.isAtTarget(2.0)) break;
            checkEmergency();
            sleep(20);
        }

        // Brief settle
        sleep(500);
        turret.resetPID(); // Clear any accumulated integral from the move to start

        if (!opModeIsActive()) { testInProgress = false; return; }

        // --- Phase 2: Command step and record ---
        testData.clear();
        ElapsedTime recordTimer = new ElapsedTime();
        recordTimer.reset();

        // Command the step
        turret.setTargetAngle(TEST_ANGLE);

        // Record data points
        while (opModeIsActive() && recordTimer.seconds() < RECORD_DURATION_SEC) {
            turret.update();

            testData.add(new DataPoint(recordTimer.seconds(), turret.getCurrentAngle()));

            telemetry.clearAll();
            telemetry.addLine("=== RECORDING TEST " + iteration + "/" + MAX_ITERATIONS + " ===");
            telemetry.addData("Time", "%.2fs / %.1fs", recordTimer.seconds(), RECORD_DURATION_SEC);
            telemetry.addData("Position", "%.1f° → target %.1f°", turret.getCurrentAngle(), TEST_ANGLE);
            telemetry.addData("Error", "%.1f°", turret.getError());
            telemetry.addData("Power", "%.3f", turret.getMotorPower());
            telemetry.addData("Samples", "%d", testData.size());
            telemetry.update();

            // Safety: abort if out of range
            if (turret.getCurrentAngle() < -5 || turret.getCurrentAngle() > 335) {
                turret.emergencyStop();
                telemetry.addLine("!! ABORTED: OUT OF RANGE !!");
                telemetry.update();
                sleep(2000);
                turret.clearEmergencyStop();
                testInProgress = false;
                return;
            }

            // Safety: abort if stalled
            if (turret.isStalled()) {
                telemetry.addLine("!! ABORTED: MOTOR STALL DETECTED !!");
                telemetry.update();
                sleep(2000);
                turret.clearStall();
                testInProgress = false;
                return;
            }

            checkEmergency();
            sleep(SAMPLE_INTERVAL_MS);
        }

        if (!opModeIsActive()) { testInProgress = false; return; }

        // --- Phase 3: Analyze response ---
        analyzeResponse();

        // --- Phase 4: Adjust gains ---
        adjustGains();

        // --- Phase 5: Return to start ---
        turret.setTargetAngle(START_ANGLE);
        turret.resetPID();
        phaseTimer.reset();
        while (opModeIsActive() && phaseTimer.seconds() < 3.0) {
            turret.update();
            if (turret.isAtTarget(3.0)) break;
            sleep(20);
        }

        // Check if tuning is complete
        if (iteration >= MAX_ITERATIONS) {
            tuningComplete = true;
        } else if (lastOvershoot >= 0 && lastSettlingTime >= 0 && lastSSError >= 0) {
            tuningComplete = (lastOvershoot < MAX_ACCEPTABLE_OVERSHOOT_DEG)
                          && (lastSettlingTime < MAX_ACCEPTABLE_SETTLING_SEC)
                          && (lastSSError < MAX_ACCEPTABLE_SS_ERROR_DEG)
                          && (lastOscillations <= MAX_ACCEPTABLE_OSCILLATIONS);
        }

        testInProgress = false;
    }

    // ========================== RESPONSE ANALYSIS ==========================

    /**
     * Analyze the recorded step response data.
     *
     * Metrics computed:
     *   Rise time:     Time to first reach 90% of the way from start to target.
     *   Overshoot:     Maximum deviation past the target (in degrees).
     *   Settling time: Time after which the error stays within ±2° permanently.
     *   SS error:      Average error over the last 0.5 seconds of recording.
     *   Oscillations:  Number of times the error crosses zero.
     */
    private void analyzeResponse() {
        if (testData.isEmpty()) {
            lastRiseTime = lastOvershoot = lastSettlingTime = lastSSError = -1;
            lastOscillations = -1;
            return;
        }

        double startAngle = testData.get(0).angleDeg;
        double targetAngle = TEST_ANGLE;
        double totalStep = targetAngle - startAngle; // Negative for our 165→50 step
        double ninetyPercent = startAngle + 0.9 * totalStep;

        // --- Rise time (time to reach 90% of step) ---
        lastRiseTime = -1;
        for (DataPoint dp : testData) {
            // For negative step: position must go BELOW ninetyPercent
            if (totalStep < 0) {
                if (dp.angleDeg <= ninetyPercent) {
                    lastRiseTime = dp.timeSec;
                    break;
                }
            } else {
                if (dp.angleDeg >= ninetyPercent) {
                    lastRiseTime = dp.timeSec;
                    break;
                }
            }
        }

        // --- Overshoot (max deviation past target) ---
        lastOvershoot = 0;
        for (DataPoint dp : testData) {
            double overshoot;
            if (totalStep < 0) {
                // Negative step: overshoot = how far BELOW target
                overshoot = targetAngle - dp.angleDeg;
            } else {
                // Positive step: overshoot = how far ABOVE target
                overshoot = dp.angleDeg - targetAngle;
            }
            if (overshoot > lastOvershoot) {
                lastOvershoot = overshoot;
            }
        }

        // --- Settling time (last time error exceeds ±2°) ---
        lastSettlingTime = RECORD_DURATION_SEC; // Default: never settled
        double settleThreshold = 2.0;
        // Walk backward from end to find last point outside threshold
        for (int i = testData.size() - 1; i >= 0; i--) {
            DataPoint dp = testData.get(i);
            double error = Math.abs(dp.angleDeg - targetAngle);
            if (error > settleThreshold) {
                // Settling time is the NEXT sample after this one
                if (i + 1 < testData.size()) {
                    lastSettlingTime = testData.get(i + 1).timeSec;
                }
                break;
            }
            if (i == 0) {
                // All samples within threshold — settled immediately
                lastSettlingTime = 0;
            }
        }

        // --- Steady-state error (average error over last 0.5s) ---
        double ssSum = 0;
        int ssCount = 0;
        double ssWindow = RECORD_DURATION_SEC - 0.5;
        for (DataPoint dp : testData) {
            if (dp.timeSec >= ssWindow) {
                ssSum += Math.abs(dp.angleDeg - targetAngle);
                ssCount++;
            }
        }
        lastSSError = (ssCount > 0) ? (ssSum / ssCount) : -1;

        // --- Oscillation count (zero-crossings of error signal) ---
        lastOscillations = 0;
        double prevError = 0;
        boolean firstSample = true;
        // Only count after rise time (ignore the initial approach)
        double countAfter = (lastRiseTime > 0) ? lastRiseTime : 0.5;
        for (DataPoint dp : testData) {
            if (dp.timeSec < countAfter) continue;
            double error = dp.angleDeg - targetAngle;
            if (!firstSample && prevError != 0 && error != 0) {
                // Sign change = zero crossing
                if ((prevError > 0 && error < 0) || (prevError < 0 && error > 0)) {
                    lastOscillations++;
                }
            }
            prevError = error;
            firstSample = false;
        }
    }

    // ========================== GAIN ADJUSTMENT ==========================

    /**
     * Adjust PID gains based on the step response analysis.
     *
     * Uses a simplified Ziegler-Nichols-inspired heuristic:
     *
     * - OSCILLATING (>3 zero-crossings):
     *     System is underdamped with sustained oscillation.
     *     Fix: Decrease P by 30% (less aggressive), increase D by 100% (more damping).
     *
     * - OVERSHOOT >10° (underdamped, but not oscillating):
     *     P is too aggressive for the current damping.
     *     Fix: Decrease P by 20%, increase D by 50%.
     *
     * - OVERDAMPED (overshoot <2° AND rise time >1.5s):
     *     System is too sluggish, needs more aggression.
     *     Fix: Increase P by 30%.
     *
     * - STEADY-STATE ERROR >3°:
     *     P can't fully close the gap (friction, gravity, etc.).
     *     Fix: Add/increase I term. I is set to 10% of P as a starting heuristic,
     *          then increased by 50% on subsequent iterations.
     *
     * All gains are clamped to sane ranges to prevent runaway tuning:
     *   P: [0.001, 0.10]    — below 0.001 is useless, above 0.1 will oscillate wildly
     *   I: [0, 0.005]       — high-ratio gearbox needs very little I
     *   D: [0, 0.05]        — too much D amplifies encoder noise
     */
    private void adjustGains() {
        // Guard — no data
        if (lastOvershoot < 0 || lastSSError < 0) return;

        // Priority 1: Fix oscillation (most dangerous)
        if (lastOscillations > MAX_ACCEPTABLE_OSCILLATIONS) {
            tuneP *= 0.70;   // Reduce P by 30%
            tuneD *= 2.00;   // Double D for damping
            // If D was zero, seed it
            if (tuneD < 0.0005) tuneD = tuneP * 0.3;
        }
        // Priority 2: Fix large overshoot
        else if (lastOvershoot > 10.0) {
            tuneP *= 0.80;   // Reduce P by 20%
            tuneD *= 1.50;   // Increase D by 50%
            if (tuneD < 0.0005) tuneD = tuneP * 0.2;
        }
        // Priority 3: Fix moderate overshoot
        else if (lastOvershoot > MAX_ACCEPTABLE_OVERSHOOT_DEG) {
            tuneP *= 0.90;   // Reduce P by 10%
            tuneD *= 1.30;   // Increase D by 30%
            if (tuneD < 0.0005) tuneD = tuneP * 0.15;
        }
        // Priority 4: Too slow (overdamped)
        else if (lastOvershoot < 2.0 && lastRiseTime > 1.5) {
            tuneP *= 1.30;   // Increase P by 30%
        }
        // Priority 5: A bit slow but acceptable overshoot
        else if (lastSettlingTime > MAX_ACCEPTABLE_SETTLING_SEC && lastOvershoot < 3.0) {
            tuneP *= 1.15;   // Gentle P increase
        }

        // Steady-state error fix (independent of above)
        if (lastSSError > 3.0) {
            if (tuneI < 0.00001) {
                // Seed I at 10% of P — usually a reasonable starting ratio
                tuneI = tuneP * 0.10;
            } else {
                tuneI *= 1.50; // Increase by 50%
            }
        } else if (lastSSError < 1.0 && tuneI > 0.0001) {
            // SS error is fine and I is high — bleed it down to prevent overshoot
            tuneI *= 0.80;
        }

        // Clamp all gains to safe ranges
        tuneP = Math.max(0.001, Math.min(tuneP, 0.10));
        tuneI = Math.max(0.0, Math.min(tuneI, 0.005));
        tuneD = Math.max(0.0, Math.min(tuneD, 0.05));
    }

    // ========================== TELEMETRY ==========================

    /** Display the main tuner UI. */
    private void displayTelemetry() {
        telemetry.clearAll();

        telemetry.addLine("=== TURRET PID AUTO-TUNER ===");
        telemetry.addLine("");

        if (tuningComplete) {
            telemetry.addLine("** TUNING COMPLETE **");
            telemetry.addLine("Press B to accept and save");
            telemetry.addLine("Press X to reset and restart");
            telemetry.addLine("");
        }

        telemetry.addData("Iteration", "%d / %d", iteration, MAX_ITERATIONS);
        telemetry.addData("Current Gains",
            "P=%.5f  I=%.6f  D=%.5f", tuneP, tuneI, tuneD);
        telemetry.addLine("");

        telemetry.addData("Turret Position", "%.1f°", turret.getCurrentAngle());
        telemetry.addData("Encoder", "%d ticks", turret.getEncoderTicks());
        telemetry.addLine("");

        if (iteration > 0 && lastOvershoot >= 0) {
            telemetry.addLine("--- Last Test Results ---");
            telemetry.addData("Rise Time",
                "%.2fs %s", lastRiseTime,
                (lastRiseTime >= 0 && lastRiseTime < 1.0) ? "(GOOD)" : "(SLOW)");
            telemetry.addData("Overshoot",
                "%.1f° %s", lastOvershoot,
                (lastOvershoot < MAX_ACCEPTABLE_OVERSHOOT_DEG) ? "(GOOD)" : "(HIGH)");
            telemetry.addData("Settling",
                "%.2fs %s", lastSettlingTime,
                (lastSettlingTime < MAX_ACCEPTABLE_SETTLING_SEC) ? "(GOOD)" : "(SLOW)");
            telemetry.addData("SS Error",
                "%.1f° %s", lastSSError,
                (lastSSError < MAX_ACCEPTABLE_SS_ERROR_DEG) ? "(GOOD)" : "(HIGH)");
            telemetry.addData("Oscillations",
                "%d %s", lastOscillations,
                (lastOscillations <= MAX_ACCEPTABLE_OSCILLATIONS) ? "(GOOD)" : "(HIGH)");
            telemetry.addLine("");
        }

        telemetry.addLine("[A] Run next test");
        telemetry.addLine("[B] Accept and save gains");
        telemetry.addLine("[X] Reset and restart");
        telemetry.addLine("[Y] EMERGENCY STOP");

        telemetry.update();
    }

    // ========================== SAVE / RESET ==========================

    /**
     * Save tuning results to a CSV file on the robot controller.
     * File: /sdcard/FIRST/turret_pid_tuning.csv
     *
     * Appends a row for each accepted tuning session so you can compare
     * results across multiple runs.
     */
    private void saveResults() {
        String path = "/sdcard/FIRST/turret_pid_tuning.csv";
        try {
            PrintWriter pw = new PrintWriter(new FileWriter(path, true)); // append mode
            // Header (written if file is new — but we just always append)
            // iteration, P, I, D, overshoot, settling, ssError, oscillations
            pw.printf("%d, %.6f, %.6f, %.6f, %.2f, %.2f, %.2f, %d%n",
                iteration, tuneP, tuneI, tuneD,
                lastOvershoot, lastSettlingTime, lastSSError, lastOscillations);
            pw.flush();
            pw.close();
        } catch (IOException e) {
            telemetry.addData("Save Error", e.getMessage());
        }

        // Also save the latest raw test data
        String dataPath = "/sdcard/FIRST/turret_pid_last_test.csv";
        try {
            PrintWriter pw = new PrintWriter(new FileWriter(dataPath, false));
            pw.println("timeSec, angleDeg");
            for (DataPoint dp : testData) {
                pw.printf("%.4f, %.2f%n", dp.timeSec, dp.angleDeg);
            }
            pw.flush();
            pw.close();
        } catch (IOException e) {
            telemetry.addData("Data Save Error", e.getMessage());
        }
    }

    /** Reset tuner to initial state for a fresh start. */
    private void resetTuner() {
        tuneP = 0.010;
        tuneI = 0.0000;
        tuneD = 0.0000;
        iteration = 0;
        tuningComplete = false;
        lastRiseTime = lastOvershoot = lastSettlingTime = lastSSError = -1;
        lastOscillations = -1;
        testData.clear();

        turret.clearEmergencyStop();
        turret.clearStall();
        turret.setPIDGains(tuneP, tuneI, tuneD);
        turret.setTargetAngle(START_ANGLE);
    }

    /** Check for emergency stop via Y button. */
    private void checkEmergency() {
        if (gamepad1.y) {
            turret.emergencyStop();
        }
    }
}
