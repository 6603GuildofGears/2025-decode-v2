package org.firstinspires.ftc.teamcode.pedroPathing.Pipelines;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.pedroPathing.TeleOp.TurretConfig;

/**
 * Turret subsystem — built from first principles.
 *
 * Coordinate system:
 *   0° = robot forward. All internal angles are robot-relative degrees.
 *   The magnetic limit switch resets the encoder to 0, but the magnet is NOT
 *   at forward — it sits at ZERO_OFFSET_TICKS from forward. So:
 *       turretAngleDeg = (encoderTicks - ZERO_OFFSET_TICKS) / TICKS_PER_DEG
 *   Raw encoder ticks are NEVER used in PID, limits, or field-lock math.
 *
 * Camera is mounted ON the turret. Therefore:
 *   - When target is visible, tx IS the error (no heading math needed)
 *   - When target is lost, use stored field angle + robot heading to hold position
 *
 * Three states: IDLE → TRACKING → LOCKED
 *
 * Call setInputs() then update() every loop. That's it.
 */
public class TurretTest {

    // ===== States =====
    public enum State { IDLE, TRACKING, LOCKED }
    private State state = State.IDLE;

    // ===== Hardware =====
    private final DcMotorEx motor;

    // ===== Encoder-to-degree constants =====
    private static final double TICKS_PER_DEG = 2.65;

    // The magnetic limit switch is at -150 ticks from robot-forward.
    // Encoder resets to 0 at the magnet, so forward (0°) = encoder tick +150.
    // Formula: turretAngleDeg = (encoderTicks - FORWARD_OFFSET_TICKS) / TICKS_PER_DEG
    // Verify on your robot and adjust if needed.
    private static final int FORWARD_OFFSET_TICKS = 150;

    // All PID gains, limits, and soft-limit values are read LIVE from
    // TurretConfig each loop so they can be tuned via Panels.

    // ===== Inputs (set each loop by caller) =====
    private boolean targetValid    = false;
    private double  tx             = 0;
    private double  robotHeadingDeg = 0;

    // ===== PID state =====
    private double integral   = 0;
    private double lastError  = 0;
    private double power      = 0;
    private boolean onTarget  = false;

    // ===== Lock state =====
    private double  lockedFieldAngle = 0;
    private boolean hasLock          = false;

    // ===== Timing =====
    private final ElapsedTime timer = new ElapsedTime();

    // ===== Telemetry =====
    private double errorDeg = 0;
    private double loopHz   = 0;
    private double pTerm    = 0;
    private double iTerm    = 0;
    private double dTerm    = 0;

    // ======================================================================
    //  Constructor
    // ======================================================================

    /**
     * @param turretMotor  Caller must init motor, set direction, reset encoder,
     *                     and set run mode BEFORE passing it here.
     */
    public TurretTest(DcMotorEx turretMotor) {
        this.motor = turretMotor;
        timer.reset();
    }

    // ======================================================================
    //  Inputs — call BEFORE update() each loop
    // ======================================================================

    /**
     * Feed sensor data into the subsystem.
     *
     * @param targetValid   true if Limelight sees the AprilTag this frame
     * @param tx            horizontal offset in degrees (positive = target right of centre)
     * @param robotHeading  robot yaw in degrees from Pinpoint/IMU (continuous, not wrapped)
     */
    public void setInputs(boolean targetValid, double tx, double robotHeading) {
        this.targetValid    = targetValid;
        this.tx             = tx;
        this.robotHeadingDeg = robotHeading;
    }

    // ======================================================================
    //  Main update — call once per loop
    // ======================================================================

    public void update() {

        // --- Timing ---
        double dt = timer.seconds();
        timer.reset();
        if (dt <= 0 || dt > 0.5) dt = 0.02;   // guard first frame / stall
        loopHz = 1.0 / dt;

        // --- Turret angle from encoder ---
        double turretAngleDeg = ticksToDeg(motor.getCurrentPosition());

        // --- State transitions ---
        if (targetValid) {
            state = State.TRACKING;
        } else if (hasLock) {
            state = State.LOCKED;
        } else {
            state = State.IDLE;
        }

        // --- Compute error based on state ---
        switch (state) {

            case TRACKING:
                // Camera is on turret → tx IS the error, no heading math
                errorDeg = tx;

                // While centred, capture the field-space lock angle
                if (Math.abs(tx) < TurretConfig.TT_LOCK_THRESH) {
                    lockedFieldAngle = normalize(robotHeadingDeg + turretAngleDeg);
                    hasLock = true;
                }
                break;

            case LOCKED:
                // Target lost — hold the last known field angle
                double desiredTurretAngle = normalize(lockedFieldAngle - robotHeadingDeg);
                errorDeg = normalize(desiredTurretAngle - turretAngleDeg);
                break;

            case IDLE:
            default:
                // Nothing to do — stop motor, bleed state
                errorDeg  = 0;
                power     = 0;
                integral  = 0;
                lastError = 0;
                pTerm = iTerm = dTerm = 0;
                onTarget  = false;
                motor.setPower(0);
                return;
        }

        // --- Deadband ---
        if (Math.abs(errorDeg) < TurretConfig.TT_DEADBAND) {
            power    = 0;
            onTarget = true;
            integral *= 0.90;   // bleed integral to prevent drift
            pTerm = iTerm = dTerm = 0;
            motor.setPower(0);
            lastError = errorDeg;
            return;
        }
        onTarget = false;

        // --- PID ---
        pTerm = TurretConfig.TT_KP * errorDeg;

        integral += errorDeg * dt;
        integral  = Range.clip(integral, -TurretConfig.TT_MAX_INTEGRAL, TurretConfig.TT_MAX_INTEGRAL);
        iTerm = TurretConfig.TT_KI * integral;

        dTerm = (dt > 0) ? TurretConfig.TT_KD * ((errorDeg - lastError) / dt) : 0;

        double output = pTerm + iTerm + dTerm;

        // --- Static friction compensation ---
        if (output != 0) {
            output += Math.signum(output) * TurretConfig.TT_K_STATIC;
        }

        // --- Clamp ---
        output = Range.clip(output, -TurretConfig.TT_MAX_POWER, TurretConfig.TT_MAX_POWER);

        // --- Soft-limits (in degrees, robot-relative) ---
        output = applySoftLimits(output, turretAngleDeg);

        // --- Apply ---
        power = output;
        motor.setPower(power);
        lastError = errorDeg;
    }

    // ======================================================================
    //  Soft-limit helper (operates entirely in robot-relative degrees)
    // ======================================================================

    private double applySoftLimits(double pwr, double angleDeg) {

        double minDeg = TurretConfig.TT_LIMIT_MIN;
        double maxDeg = TurretConfig.TT_LIMIT_MAX;
        double slow   = TurretConfig.TT_SLOW_ZONE;

        // Hard kill past limits
        if (angleDeg <= minDeg && pwr < 0) return 0;
        if (angleDeg >= maxDeg && pwr > 0) return 0;

        // Linear ramp-down inside slow zone
        if (angleDeg < minDeg + slow && pwr < 0) {
            double scale = (angleDeg - minDeg) / slow;
            pwr *= Range.clip(scale, 0.05, 1.0);
        }
        if (angleDeg > maxDeg - slow && pwr > 0) {
            double scale = (maxDeg - angleDeg) / slow;
            pwr *= Range.clip(scale, 0.05, 1.0);
        }
        return pwr;
    }

    // ======================================================================
    //  Angle helpers
    // ======================================================================

    /** Normalize any angle to [−180, +180]. */
    private static double normalize(double deg) {
        deg = deg % 360;
        if (deg > 180)  deg -= 360;
        if (deg < -180) deg += 360;
        return deg;
    }

    /** Encoder ticks → robot-relative turret degrees (0° = forward). */
    private static double ticksToDeg(int ticks) {
        return (ticks - FORWARD_OFFSET_TICKS) / TICKS_PER_DEG;
    }

    /** Robot-relative degrees → encoder ticks. */
    private static int degToTicks(double deg) {
        return (int) Math.round(deg * TICKS_PER_DEG + FORWARD_OFFSET_TICKS);
    }

    // ======================================================================
    //  Reset / control
    // ======================================================================

    /** Full reset — PID state, lock, timer. Call at start(). */
    public void reset() {
        integral  = 0;
        lastError = 0;
        power     = 0;
        errorDeg  = 0;
        pTerm = iTerm = dTerm = 0;
        onTarget  = false;
        hasLock   = false;
        lockedFieldAngle = 0;
        state     = State.IDLE;
        timer.reset();
    }

    /**
     * Zero the encoder at the current position (call when magnet sensor is hit).
     * Also clears PID state so stale integral/derivative don't cause a jerk.
     */
    public void zeroEncoder() {
        motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        integral  = 0;
        lastError = 0;
        power     = 0;
        hasLock   = false;
        lockedFieldAngle = 0;
    }

    /**
     * Apply soft-limits to an arbitrary power value using the current turret angle.
     * Use this from manual-control code paths that bypass update().
     */
    public double limitPower(double pwr) {
        return applySoftLimits(pwr, ticksToDeg(motor.getCurrentPosition()));
    }

    /** Drop the lock (return to IDLE when target lost). */
    public void clearLock() {
        hasLock = false;
        lockedFieldAngle = 0;
    }

    // ======================================================================
    //  Getters — telemetry
    // ======================================================================

    public State   getState()      { return state; }
    public double  getPower()      { return power; }
    public double  getError()      { return errorDeg; }
    public boolean isOnTarget()    { return onTarget; }
    public boolean hasLock()       { return hasLock; }
    public double  getLockedAngle(){ return lockedFieldAngle; }
    /** Raw encoder ticks — debug only, do not use for control math. */
    public int     getEncoderRaw() { return motor.getCurrentPosition(); }
    /** Turret angle in robot-relative degrees (0° = forward). */
    public double  getTurretAngle(){ return ticksToDeg(motor.getCurrentPosition()); }
    public double  getLoopHz()     { return loopHz; }
    public double  getPTerm()      { return pTerm; }
    public double  getITerm()      { return iTerm; }
    public double  getDTerm()      { return dTerm; }
    public double  getIntegral()   { return integral; }
    public double  getKP()         { return TurretConfig.TT_KP; }
    public double  getKI()         { return TurretConfig.TT_KI; }
    public double  getKD()         { return TurretConfig.TT_KD; }
}
