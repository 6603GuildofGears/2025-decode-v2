package org.firstinspires.ftc.teamcode.pedroPathing.Pipelines;

import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import static org.firstinspires.ftc.teamcode.pedroPathing.Pipelines.Motor_PipeLine.turret;
import static org.firstinspires.ftc.teamcode.pedroPathing.TeleOp.TurretConfig.*;

/**
 * Turret PID controller — rebuilt from scratch.
 *
 * Reads gains and limits from TurretConfig (live-tunable via Panels).
 * Drives Motor_PipeLine.turret to centre on a Limelight AprilTag.
 *
 * Features:
 *   - Reads KP/KI/KD from TurretConfig every loop (hot-tunable)
 *   - Low-pass filter on the TX input (configurable alpha)
 *   - Integral anti-windup with configurable cap
 *   - Deadband (error tolerance) from TurretConfig
 *   - Max power clamp from TurretConfig
 *   - Integral decay when on-target to prevent creep
 *   - Tracks "on target" status for external logic
 *
 * Prerequisites — call BEFORE using this class:
 *   Motor_PipeLine.intMotors(opMode);
 *   Limelight_Pipeline.initLimelight(opMode);
 *
 * Usage in loop:
 *   turretPID.update(hasBlueGoal(), getBlueGoalX());
 */
public class TurretPipeline {

    // ===== Internal PID state =====
    private double lastError  = 0;
    private double integral   = 0;
    private double filteredTX = 0;   // low-pass filtered input
    private double power      = 0;
    private boolean onTarget  = false;
    private boolean firstFrame = true;

    private final ElapsedTime timer = new ElapsedTime();

    // ===== Constructor =====
    public TurretPipeline() {
        timer.reset();
    }

    // ===== Getters for telemetry =====
    public double getPower()     { return power; }
    public boolean isOnTarget()  { return onTarget; }
    public double getFiltered()  { return filteredTX; }
    public double getIntegral()  { return integral; }

    /** Reset PID state + timer. Call at opMode start(). */
    public void reset() {
        lastError  = 0;
        integral   = 0;
        filteredTX = 0;
        power      = 0;
        onTarget   = false;
        firstFrame = true;
        timer.reset();
    }

    /** Alias kept for backward compatibility with existing OpModes. */
    public void resetTimer() { reset(); }

    // Stub setters kept so Auto_Aim_Test still compiles (it is @Disabled).
    public void setKP(double v) {}
    public double getKP() { return KP_TURRET; }
    public void setKI(double v) {}
    public double getKI() { return KI_TURRET; }
    public void setKD(double v) {}
    public double getKD() { return KD_TURRET; }
    public void setErrorTolerance(double v) {}
    public double getErrorTolerance() { return TURRET_DEADBAND; }

    // =================================================================
    //  MAIN UPDATE — call every loop iteration
    // =================================================================

    /**
     * @param hasTarget  true if Limelight sees the desired AprilTag
     * @param rawTX      horizontal offset in degrees (positive = target right of centre)
     */
    public void update(boolean hasTarget, double rawTX) {

        // --- Delta time ---
        double dt = timer.seconds();
        timer.reset();
        if (dt <= 0 || dt > 0.5) dt = 0.02; // guard against first-frame / stalls

        // --- No target: coast to zero, bleed state ---
        if (!hasTarget) {
            power     = 0;
            lastError = 0;
            integral  = 0;
            onTarget  = false;
            firstFrame = true;
            turret.setPower(0);
            return;
        }

        // --- Low-pass filter on TX ---
        if (firstFrame) {
            filteredTX = rawTX;   // seed filter with first reading
            firstFrame = false;
        } else {
            filteredTX = FILTER_ALPHA * rawTX + (1.0 - FILTER_ALPHA) * filteredTX;
        }

        double error = filteredTX;

        // --- Deadband check ---
        if (Math.abs(error) < TURRET_DEADBAND) {
            power    = 0;
            onTarget = true;
            integral *= 0.9;  // slowly bleed integral to prevent wind-up
            turret.setPower(0);
            lastError = error;
            return;
        }

        onTarget = false;

        // --- P term ---
        double pTerm = KP_TURRET * error;

        // --- I term with anti-windup clamp ---
        integral += error * dt;
        integral  = Range.clip(integral, -MAX_INTEGRAL, MAX_INTEGRAL);
        double iTerm = KI_TURRET * integral;

        // --- D term ---
        double dTerm = KD_TURRET * ((error - lastError) / dt);

        // --- Combine & clamp ---
        power = Range.clip(pTerm + iTerm + dTerm, -MAX_TURRET_SPEED, MAX_TURRET_SPEED);

        turret.setPower(power);
        lastError = error;
    }
}