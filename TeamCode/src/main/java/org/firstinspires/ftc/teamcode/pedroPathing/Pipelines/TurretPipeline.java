package org.firstinspires.ftc.teamcode.pedroPathing.Pipelines;

import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import static org.firstinspires.ftc.teamcode.pedroPathing.Pipelines.Motor_PipeLine.turret;

/**
 * Turret PID controller for AprilTag tracking.
 * Uses the static turret motor from Motor_PipeLine.
 *
 * Prerequisites — call these BEFORE using this class:
 *   Motor_PipeLine.intMotors(opMode);
 *   Limelight_Pipeline.initLimelight(opMode);
 *
 * Usage in loop:
 *   turretPID.update(hasBlueGoal(), getBlueGoalX());
 */
public class TurretPipeline {

    private double KP = 0.01300;
    private double KI = 0.00100;
    private double KD = 0.00110;

    private double lastError = 0;
    private double integral = 0;
    private double errorTolerance = 0.35;

    private final double MAX_POWER = 0.6;

    private double power = 0;

    private final ElapsedTime timer = new ElapsedTime();

    public TurretPipeline() {
        timer.reset();
    }

    // --- Setters / Getters ---

    public void setKP(double newKP) { KP = newKP; }
    public double getKP() { return KP; }

    public void setKI(double newKI) { KI = newKI; }
    public double getKI() { return KI; }

    public void setKD(double newKD) { KD = newKD; }
    public double getKD() { return KD; }

    public void setErrorTolerance(double tol) { errorTolerance = tol; }
    public double getErrorTolerance() { return errorTolerance; }

    public double getPower() { return power; }

    public void resetTimer() {
        timer.reset();
    }

    /**
     * Call every loop iteration.
     * @param hasTarget  true if Limelight sees the target
     * @param targetTX   horizontal offset in degrees (0 = centered)
     */
    public void update(boolean hasTarget, double targetTX) {
        double deltaTime = timer.seconds();
        timer.reset();

        if (!hasTarget) {
            power = 0;
            lastError = 0;
            integral = 0;
            return;
        }

        double error = targetTX;

        // P term
        double pTerm = KP * error;

        // I term with anti-windup
        integral += error * deltaTime;
        double maxIntegral = (KI != 0) ? MAX_POWER / Math.max(Math.abs(KI), 0.0001) : 0;
        integral = Range.clip(integral, -maxIntegral, maxIntegral);
        double iTerm = KI * integral;

        // D term
        double dTerm = 0;
        if (deltaTime > 0) {
            dTerm = KD * ((error - lastError) / deltaTime);
        }

        if (Math.abs(error) < errorTolerance) {
            power = 0;
            integral *= 0.95; // decay integral when on target
        } else {
            power = Range.clip(pTerm + iTerm + dTerm, -MAX_POWER, MAX_POWER);
        }

        turret.setPower(power);
        lastError = error;
    }
}