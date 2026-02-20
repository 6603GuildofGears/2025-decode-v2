package org.firstinspires.ftc.teamcode.pedroPathing.Pipelines;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

/**
 * TurretSubsystem — Robot-relative position control for a geared turret.
 *
 * ========================== HARDWARE ==========================
 *   Motor:       goBILDA 1150 RPM Yellow Jacket (5203 series)
 *   Encoder:     145.1 CPR at the motor output shaft
 *   Gear Ratio:  20:131 external reduction (motor → turret)
 *                Motor turns 6.55 times per 1 turret revolution
 *   Final:       950.405 ticks per turret revolution
 *                2.640 ticks per turret degree
 *   Max Speed:   ~175 RPM at turret = ~1050°/s theoretical
 *   Range:       0° to 330° (cable-limited, NOT continuous)
 *   Zero Sensor: Magnetic limit switch (TouchSensor "mag") at 0°
 *
 * ========================== COORDINATE SYSTEM ==========================
 *   0°   = Home position (magnetic limit switch location)
 *   330° = Maximum travel
 *   All angles are ROBOT-RELATIVE — no chassis compensation.
 *
 * ========================== TICK-TO-DEGREE MATH ==========================
 *   ticksPerMotorRev  = 145.1         (goBILDA 1150 RPM spec)
 *   externalGearRatio = 131 / 20      (motor turns per turret turn)
 *   ticksPerTurretRev = 145.1 × 6.55  = 950.405
 *   ticksPerDegree    = 950.405 / 360  = 2.640
 *
 *   degrees = encoderTicks / TICKS_PER_DEG
 *   ticks   = degrees × TICKS_PER_DEG
 *
 * ========================== PID ALGORITHM ==========================
 *   error     = targetAngle - currentAngle
 *   integral += error × dt          (clamped for anti-windup)
 *   derivative = (error - lastError) / dt
 *
 *   output = (P × error) + (I × integral) + (D × derivative)
 *   output = clamp(output, -maxPower, +maxPower)
 *   output = applySoftLimits(output)
 *
 * ========================== USAGE ==========================
 *   // In init():
 *   TurretSubsystem turretSub = new TurretSubsystem(turretMotor, magSensor);
 *
 *   // In start() or init_loop():
 *   turretSub.home();    // blocks until homed (use homeAsync for non-blocking)
 *
 *   // In loop():
 *   turretSub.setTargetAngle(90);
 *   turretSub.update();
 *
 *   telemetry.addData("Angle", turretSub.getCurrentAngle());
 *   telemetry.addData("AtTarget", turretSub.isAtTarget(2.0));
 */
public class TurretSubsystem {

    // ========================== ENCODER CONSTANTS ==========================
    // goBILDA 1150 RPM motor: 145.1 encoder counts per motor output revolution
    private static final double TICKS_PER_MOTOR_REV = 145.1;

    // External gear ratio: 20-tooth driving 131-tooth = 6.55:1 reduction
    // Motor must turn 6.55 times for turret to turn once
    private static final double EXTERNAL_GEAR_RATIO = 131.0 / 20.0; // = 6.55

    // Combined: ticks per full turret revolution
    // 145.1 × 6.55 = 950.405 ticks/rev
    private static final double TICKS_PER_TURRET_REV = TICKS_PER_MOTOR_REV * EXTERNAL_GEAR_RATIO;

    // Ticks per degree of turret rotation
    // 950.405 / 360 = 2.640 ticks/degree
    public static final double TICKS_PER_DEG = TICKS_PER_TURRET_REV / 360.0;

    // ========================== MECHANICAL LIMITS ==========================
    /** Minimum turret angle in degrees (home / limit switch position). */
    public static final double MIN_ANGLE_DEG = 0.0;

    /** Maximum turret angle in degrees (cable-limited, with safety margin). */
    public static final double MAX_ANGLE_DEG = 320.0;

    /** Center of the turret's range — default position after homing. */
    public static final double CENTER_ANGLE_DEG = 165.0;

    /** Degrees from the hard limits where speed ramps down linearly. */
    private static final double SLOW_ZONE_DEG = 20.0;

    // ========================== PID GAINS ==========================
    // These are starting defaults. The auto-tuner will find better values.
    // Expected range for this high-ratio turret:
    //   P: 0.008 – 0.025  (high gear ratio amplifies motor response)
    //   I: 0.0001 – 0.001 (high torque means minimal SS error)
    //   D: 0.001 – 0.008  (damping needed due to inertia)
    private double kP = 0.015;
    private double kI = 0.0003;
    private double kD = 0.003;

    /** Maximum I accumulator magnitude (degree-seconds). Prevents windup. */
    private double maxIntegral = 50.0;

    /** Maximum motor power (0–1). Start conservative for tuning. */
    private double maxPower = 0.7;

    /** Position tolerance in degrees. Below this error → "on target". */
    private double defaultTolerance = 2.0;

    // ========================== HOMING ==========================
    /** Motor power used during homing (very slow to not overshoot the magnet). */
    private static final double HOMING_POWER = -0.10;

    /** After encoder resets at magnet, nudge this many degrees off the switch. */
    private static final double HOMING_CLEAR_DEG = 5.0;

    // ========================== STALL DETECTION ==========================
    /** If motor is commanded but encoder doesn't change for this long (ms), flag stall. */
    private static final long STALL_TIMEOUT_MS = 500;

    /** Minimum encoder change (ticks) to consider "not stalled". */
    private static final int STALL_TICK_THRESHOLD = 3;

    // ========================== HARDWARE ==========================
    private final DcMotorEx motor;
    private final TouchSensor magSwitch;

    // ========================== STATE ==========================
    private double targetAngle = 0;     // Commanded angle in degrees
    private double currentAngle = 0;    // Current angle in degrees (updated each loop)

    // PID state
    private double integral = 0;
    private double lastError = 0;
    private double motorPower = 0;
    private boolean onTarget = false;

    // Homing state
    private boolean homed = false;

    // Stall detection
    private int lastEncoderPos = 0;
    private long lastEncoderChangeTime = 0;
    private boolean stalled = false;

    // Safety
    private boolean emergencyStopped = false;

    // Timing
    private final ElapsedTime pidTimer = new ElapsedTime();

    // ========================== CONSTRUCTOR ==========================

    /**
     * Create a new TurretSubsystem.
     *
     * @param turretMotor  DcMotorEx for the turret. Direction should already be set
     *                     by Motor_PipeLine. Encoder will be reset during homing.
     * @param magnetSwitch TouchSensor for the magnetic limit switch at 0°.
     */
    public TurretSubsystem(DcMotorEx turretMotor, TouchSensor magnetSwitch) {
        this.motor = turretMotor;
        this.magSwitch = magnetSwitch;

        // Ensure motor is in the right mode for manual power control with encoder reading
        motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        pidTimer.reset();
    }

    // ========================== PUBLIC API ==========================

    /**
     * Command the turret to move to the specified angle.
     * Angle is clamped to [MIN_ANGLE_DEG, MAX_ANGLE_DEG].
     *
     * @param degrees Target angle in turret degrees (0° = home, 330° = max).
     */
    public void setTargetAngle(double degrees) {
        if (emergencyStopped) return;
        targetAngle = Range.clip(degrees, MIN_ANGLE_DEG, MAX_ANGLE_DEG);
    }

    /**
     * Get the current turret angle in degrees.
     * Updated every time update() is called.
     */
    public double getCurrentAngle() {
        return currentAngle;
    }

    /**
     * Check if the turret is within the specified tolerance of its target.
     *
     * @param toleranceDeg Acceptable error in degrees.
     * @return true if |error| < tolerance.
     */
    public boolean isAtTarget(double toleranceDeg) {
        return Math.abs(targetAngle - currentAngle) < toleranceDeg;
    }

    /**
     * Check if the turret is at target using the default tolerance (2°).
     */
    public boolean isAtTarget() {
        return isAtTarget(defaultTolerance);
    }

    /**
     * Get the current position error in degrees (target - current).
     */
    public double getError() {
        return targetAngle - currentAngle;
    }

    /**
     * Get the last computed motor power output.
     */
    public double getMotorPower() {
        return motorPower;
    }

    /**
     * Check if the turret has been successfully homed.
     */
    public boolean isHomed() {
        return homed;
    }

    /**
     * Check if stall has been detected.
     */
    public boolean isStalled() {
        return stalled;
    }

    /**
     * Check if emergency stop is active.
     */
    public boolean isEmergencyStopped() {
        return emergencyStopped;
    }

    /**
     * Get the raw encoder position in ticks.
     */
    public int getEncoderTicks() {
        return motor.getCurrentPosition();
    }

    /**
     * Get the current target angle.
     */
    public double getTargetAngle() {
        return targetAngle;
    }

    // ========================== PID GAIN CONTROL ==========================

    /**
     * Set all PID gains at once. Used by the auto-tuner.
     */
    public void setPIDGains(double p, double i, double d) {
        this.kP = p;
        this.kI = i;
        this.kD = d;
        // Reset integral when gains change to prevent accumulated error from old gains
        integral = 0;
    }

    public double getKP() { return kP; }
    public double getKI() { return kI; }
    public double getKD() { return kD; }

    /**
     * Set the maximum motor power (0–1). Lower for safer tuning.
     */
    public void setMaxPower(double max) {
        this.maxPower = Range.clip(Math.abs(max), 0, 1.0);
    }

    public double getMaxPower() { return maxPower; }

    /**
     * Set the default on-target tolerance in degrees.
     */
    public void setDefaultTolerance(double deg) {
        this.defaultTolerance = Math.abs(deg);
    }

    /**
     * Set the anti-windup integral cap (degree-seconds).
     */
    public void setMaxIntegral(double cap) {
        this.maxIntegral = Math.abs(cap);
    }

    // ========================== HOMING ==========================

    /**
     * BLOCKING homing routine. Slowly rotates counterclockwise until the magnetic
     * limit switch triggers, then resets the encoder to 0.
     *
     * Call this during init or start. It will block the calling thread until complete
     * or until timeout (5 seconds).
     *
     * After homing, targets the center of the range (165°).
     *
     * @return true if homing succeeded, false if timed out.
     */
    public boolean home() {
        homed = false;
        stalled = false;
        emergencyStopped = false;
        integral = 0;
        lastError = 0;

        ElapsedTime homeTimer = new ElapsedTime();
        homeTimer.reset();

        // If already on the magnet, briefly move off it first so we get a clean trigger
        if (magSwitch.isPressed()) {
            motor.setPower(0.15); // Move CW (positive = away from home)
            while (magSwitch.isPressed() && homeTimer.seconds() < 2.0) {
                // Poll fast (1ms) so we don't overshoot the magnet zone
                try { Thread.sleep(1); } catch (InterruptedException e) { break; }
            }
            motor.setPower(0);
            try { Thread.sleep(100); } catch (InterruptedException e) { /* ok */ }
            homeTimer.reset();
        }

        // Now rotate counterclockwise (negative power) until magnet triggers
        motor.setPower(HOMING_POWER);

        while (!magSwitch.isPressed()) {
            // Timeout after 8 seconds (slow speed needs more time)
            if (homeTimer.seconds() > 8.0) {
                motor.setPower(0);
                return false;
            }
            // Poll every 1ms so we can't pass through the magnet zone undetected
            try { Thread.sleep(1); } catch (InterruptedException e) { break; }
        }

        // Magnet triggered — stop and reset encoder
        motor.setPower(0);

        motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        homed = true;
        currentAngle = 0;
        targetAngle = CENTER_ANGLE_DEG;

        // Reset PID state
        integral = 0;
        lastError = 0;
        motorPower = 0;
        pidTimer.reset();

        // Initialize stall detection
        lastEncoderPos = 0;
        lastEncoderChangeTime = System.currentTimeMillis();

        return true;
    }

    /**
     * Non-blocking homing state machine. Call this in a loop.
     * Returns the current homing state.
     *
     * States:
     *   CLEARING   - Moving off the magnet (if started on it)
     *   SEEKING    - Rotating CCW toward the magnet
     *   DONE       - Homing complete, encoder zeroed
     *   FAILED     - Timed out
     */
    public enum HomingState { NOT_STARTED, CLEARING, SEEKING, DONE, FAILED }
    private HomingState homingState = HomingState.NOT_STARTED;
    private ElapsedTime asyncHomeTimer;

    /**
     * Start the async homing process. Call homeAsyncUpdate() each loop afterwards.
     */
    public void homeAsyncStart() {
        homed = false;
        stalled = false;
        emergencyStopped = false;
        integral = 0;
        lastError = 0;
        asyncHomeTimer = new ElapsedTime();
        asyncHomeTimer.reset();

        if (magSwitch.isPressed()) {
            homingState = HomingState.CLEARING;
            motor.setPower(0.2);
        } else {
            homingState = HomingState.SEEKING;
            motor.setPower(HOMING_POWER);
        }
    }

    /**
     * Update the async homing state machine. Call every loop.
     * @return Current HomingState.
     */
    public HomingState homeAsyncUpdate() {
        switch (homingState) {
            case CLEARING:
                if (!magSwitch.isPressed()) {
                    // Cleared the magnet, now seek
                    homingState = HomingState.SEEKING;
                    motor.setPower(HOMING_POWER);
                    asyncHomeTimer.reset();
                } else if (asyncHomeTimer.seconds() > 2.0) {
                    motor.setPower(0);
                    homingState = HomingState.FAILED;
                }
                break;

            case SEEKING:
                if (magSwitch.isPressed()) {
                    // Found the magnet
                    motor.setPower(0);
                    motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                    motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                    homed = true;
                    currentAngle = 0;
                    targetAngle = CENTER_ANGLE_DEG;
                    integral = 0;
                    lastError = 0;
                    motorPower = 0;
                    pidTimer.reset();
                    lastEncoderPos = 0;
                    lastEncoderChangeTime = System.currentTimeMillis();
                    homingState = HomingState.DONE;
                } else if (asyncHomeTimer.seconds() > 5.0) {
                    motor.setPower(0);
                    homingState = HomingState.FAILED;
                }
                break;

            case DONE:
            case FAILED:
            case NOT_STARTED:
                // Nothing to do
                break;
        }
        return homingState;
    }

    // ========================== MAIN UPDATE LOOP ==========================

    /**
     * PID update — call this EVERY loop iteration.
     *
     * Reads encoder, computes PID, applies soft limits, commands motor.
     * Does nothing if not homed or emergency-stopped.
     */
    public void update() {
        // Read encoder and convert to degrees
        int rawTicks = motor.getCurrentPosition();
        currentAngle = ticksToDeg(rawTicks);

        // Don't run PID if not homed or e-stopped
        if (!homed || emergencyStopped) {
            motor.setPower(0);
            motorPower = 0;
            return;
        }

        // --- Emergency: limit switch triggers during normal operation ---
        // This means we've hit the physical stop unexpectedly. Re-zero.
        if (magSwitch.isPressed() && Math.abs(currentAngle) > 10) {
            // We're far from 0° but the magnet triggered — something is wrong
            // Re-zero the encoder and hold position
            motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            currentAngle = 0;
            targetAngle = HOMING_CLEAR_DEG; // Move slightly off the switch
            integral = 0;
            lastError = 0;
            pidTimer.reset();
        }

        // --- Stall detection ---
        long now = System.currentTimeMillis();
        if (Math.abs(rawTicks - lastEncoderPos) >= STALL_TICK_THRESHOLD) {
            lastEncoderPos = rawTicks;
            lastEncoderChangeTime = now;
            stalled = false;
        } else if (Math.abs(motorPower) > 0.05 && (now - lastEncoderChangeTime) > STALL_TIMEOUT_MS) {
            // Power is being commanded but encoder isn't moving
            stalled = true;
            motor.setPower(0);
            motorPower = 0;
            return;
        }

        // --- Delta time ---
        double dt = pidTimer.seconds();
        pidTimer.reset();
        if (dt <= 0 || dt > 0.5) dt = 0.02; // Guard against first frame / stalls

        // --- PID computation ---
        double error = targetAngle - currentAngle;

        // Proportional term
        double pTerm = kP * error;

        // Integral term with anti-windup clamping
        integral += error * dt;
        integral = Range.clip(integral, -maxIntegral, maxIntegral);
        double iTerm = kI * integral;

        // Derivative term
        double dTerm = (dt > 0) ? kD * ((error - lastError) / dt) : 0;

        // Combined output
        double output = pTerm + iTerm + dTerm;

        // --- Clamp to max power ---
        output = Range.clip(output, -maxPower, maxPower);

        // --- Apply soft limits ---
        output = applySoftLimits(output, currentAngle);

        // --- On-target check (bleed integral when close) ---
        onTarget = Math.abs(error) < defaultTolerance;
        if (onTarget) {
            integral *= 0.90; // Slowly bleed to prevent drift
        }

        // --- Apply to motor ---
        motorPower = output;
        motor.setPower(motorPower);
        lastError = error;
    }

    // ========================== FORCE HOMED ==========================

    /**
     * Bypass the homing routine and consider the current encoder position as 0°.
     * Use this when auto-homing fails (wrong direction, sensor issue, etc.)
     * and you want to test PID anyway.
     *
     * Resets the encoder to 0 at the current position, sets homed=true,
     * and targets the center of the range.
     */
    public void forceHomed() {
        motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        homed = true;
        currentAngle = 0;
        targetAngle = CENTER_ANGLE_DEG;
        integral = 0;
        lastError = 0;
        motorPower = 0;
        stalled = false;
        emergencyStopped = false;
        pidTimer.reset();
        lastEncoderPos = 0;
        lastEncoderChangeTime = System.currentTimeMillis();
    }

    // ========================== EMERGENCY STOP ==========================

    /**
     * Immediately stop the motor and disable all PID control.
     * Call clearEmergencyStop() to re-enable.
     */
    public void emergencyStop() {
        emergencyStopped = true;
        motor.setPower(0);
        motorPower = 0;
        integral = 0;
    }

    /**
     * Clear emergency stop and allow PID to resume.
     */
    public void clearEmergencyStop() {
        emergencyStopped = false;
        stalled = false;
        integral = 0;
        lastError = 0;
        pidTimer.reset();
        lastEncoderPos = motor.getCurrentPosition();
        lastEncoderChangeTime = System.currentTimeMillis();
    }

    /**
     * Clear the stall flag and allow PID to resume.
     */
    public void clearStall() {
        stalled = false;
        lastEncoderPos = motor.getCurrentPosition();
        lastEncoderChangeTime = System.currentTimeMillis();
    }

    /**
     * Reset PID state without changing target. Useful after manual moves.
     */
    public void resetPID() {
        integral = 0;
        lastError = 0;
        pidTimer.reset();
    }

    // ========================== SOFT LIMITS ==========================

    /**
     * Apply soft limits to motor power based on current angle.
     *
     * Hard kill:  If at/past a limit moving toward it, output = 0.
     * Ramp zone:  Within SLOW_ZONE_DEG of a limit, linearly scale power down
     *             so the turret approaches the limit gently.
     *
     * @param power     Requested motor power.
     * @param angleDeg  Current turret angle.
     * @return          Limited motor power.
     */
    private double applySoftLimits(double power, double angleDeg) {
        // Hard kill at limits
        if (angleDeg <= MIN_ANGLE_DEG && power < 0) return 0;
        if (angleDeg >= MAX_ANGLE_DEG && power > 0) return 0;

        // Linear ramp-down approaching minimum
        if (angleDeg < MIN_ANGLE_DEG + SLOW_ZONE_DEG && power < 0) {
            double scale = (angleDeg - MIN_ANGLE_DEG) / SLOW_ZONE_DEG;
            power *= Range.clip(scale, 0.05, 1.0);
        }

        // Linear ramp-down approaching maximum
        if (angleDeg > MAX_ANGLE_DEG - SLOW_ZONE_DEG && power > 0) {
            double scale = (MAX_ANGLE_DEG - angleDeg) / SLOW_ZONE_DEG;
            power *= Range.clip(scale, 0.05, 1.0);
        }

        return power;
    }

    // ========================== CONVERSION HELPERS ==========================

    /**
     * Convert encoder ticks to turret degrees.
     *
     * degrees = ticks / TICKS_PER_DEG
     *
     * With our hardware:
     *   ticksPerMotorRev  = 145.1
     *   externalRatio     = 131/20 = 6.55
     *   ticksPerTurretRev = 145.1 × 6.55 = 950.405
     *   ticksPerDeg       = 950.405 / 360 = 2.640
     *
     * So 1000 ticks = 1000 / 2.640 = 378.8°
     */
    public static double ticksToDeg(int ticks) {
        return ticks / TICKS_PER_DEG;
    }

    /**
     * Convert turret degrees to encoder ticks.
     */
    public static int degToTicks(double degrees) {
        return (int) Math.round(degrees * TICKS_PER_DEG);
    }

    // ========================== TELEMETRY HELPERS ==========================

    /** Get all PID diagnostics as a formatted string block for telemetry. */
    public String getDiagnostics() {
        return String.format(
            "Angle: %.1f° | Target: %.1f° | Error: %.1f°\n" +
            "Power: %.3f | Homed: %b | Stalled: %b\n" +
            "PID: P=%.5f I=%.6f D=%.5f\n" +
            "Encoder: %d ticks",
            currentAngle, targetAngle, getError(),
            motorPower, homed, stalled,
            kP, kI, kD,
            motor.getCurrentPosition()
        );
    }
}
