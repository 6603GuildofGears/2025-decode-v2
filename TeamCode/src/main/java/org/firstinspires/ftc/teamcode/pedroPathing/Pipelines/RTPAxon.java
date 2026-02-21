package org.firstinspires.ftc.teamcode.pedroPathing.Pipelines;

import android.annotation.SuppressLint;

import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.util.ElapsedTime;

/**
 * RTPAxon — Run-To-Position wrapper for a continuous rotation Axon servo
 * with analog encoder feedback. Gives a CR servo "go to angle" capability
 * via a built-in PID controller.
 *
 * Usage:
 *   CRServo cr = hardwareMap.crservo.get("spindexerCR");
 *   AnalogInput enc = hardwareMap.get(AnalogInput.class, "spindexerEncoder");
 *   RTPAxon axon = new RTPAxon(cr, enc);
 *
 *   axon.setTargetRotation(120);  // go to 120°
 *   // in loop:
 *   axon.update();  // MUST call every loop
 */
public class RTPAxon {

    private final AnalogInput servoEncoder;
    private final CRServo servo;

    private boolean rtp;
    private double power;
    private double maxPower;
    private Direction direction;

    private double previousAngle;
    private double totalRotation;
    private double targetRotation;

    // PID
    private double kP;
    private double kI;
    private double kD;
    private double integralSum;
    private double lastError;
    private double lastDerivative;
    private double maxIntegralSum;
    private ElapsedTime pidTimer;

    // Encoder filter (exponential moving average)
    private double filteredAngle;
    private boolean filterInitialized;
    private static final double FILTER_ALPHA = 0.3;  // 0..1; lower = smoother but laggier
    private static final double MAX_DERIVATIVE = 500; // cap derivative magnitude (°/s)

    // Init debug
    public double STARTPOS;
    public int ntry = 0;
    public int cliffs = 0;
    public double homeAngle;

    public enum Direction {
        FORWARD,
        REVERSE
    }

    // ── Constructors ──

    public RTPAxon(CRServo servo, AnalogInput encoder) {
        rtp = true;
        this.servo = servo;
        servoEncoder = encoder;
        direction = Direction.FORWARD;
        initialize();
    }

    public RTPAxon(CRServo servo, AnalogInput encoder, Direction direction) {
        this(servo, encoder);
        this.direction = direction;
        initialize();
    }

    private void initialize() {
        servo.setPower(0);
        try { Thread.sleep(50); } catch (InterruptedException ignored) {}

        // Try to get a valid starting position
        do {
            STARTPOS = getCurrentAngle();
            if (Math.abs(STARTPOS) > 1) {
                previousAngle = getCurrentAngle();
            } else {
                try { Thread.sleep(50); } catch (InterruptedException ignored) {}
            }
            ntry++;
        } while (Math.abs(previousAngle) < 0.2 && (ntry < 50));

        totalRotation = 0;
        homeAngle = previousAngle;

        // Default PID coefficients
        kP = 0.015;
        kI = 0.0005;
        kD = 0.0025;
        integralSum = 0.0;
        lastError = 0.0;
        lastDerivative = 0.0;
        maxIntegralSum = 100.0;
        pidTimer = new ElapsedTime();
        pidTimer.reset();

        // Initialize filter with current reading
        filteredAngle = getRawAngle();
        filterInitialized = true;

        maxPower = 0.25;
        cliffs = 0;
    }

    // ── Direction ──

    public void setDirection(Direction direction) { this.direction = direction; }

    // ── Power ──

    public void setPower(double power) {
        this.power = Math.max(-maxPower, Math.min(maxPower, power));
        servo.setPower(this.power * (direction == Direction.REVERSE ? -1 : 1));
    }

    public double getPower() { return power; }

    public void setMaxPower(double maxPower) { this.maxPower = maxPower; }
    public double getMaxPower() { return maxPower; }

    // ── Run-to-position mode ──

    public void setRtp(boolean rtp) {
        this.rtp = rtp;
        if (rtp) resetPID();
    }
    public boolean getRtp() { return rtp; }

    // ── PID tuning ──

    public void setKP(double kP) { this.kP = kP; }
    public void setKI(double kI) { this.kI = kI; resetIntegral(); }
    public void setKD(double kD) { this.kD = kD; }
    public void setPidCoeffs(double kP, double kI, double kD) {
        setKP(kP); setKI(kI); setKD(kD);
    }
    public double getKP() { return kP; }
    public double getKI() { return kI; }
    public double getKD() { return kD; }
    public void setMaxIntegralSum(double max) { this.maxIntegralSum = max; }

    // ── Position ──

    public double getTotalRotation() { return totalRotation; }
    public double getTargetRotation() { return targetRotation; }

    public void changeTargetRotation(double change) { targetRotation += change; }

    public void setTargetRotation(double target) {
        targetRotation = target;
        resetPID();
    }

    /** Raw unfiltered angle from analog encoder (0–360°). */
    public double getRawAngle() {
        if (servoEncoder == null) return 0;
        return (servoEncoder.getVoltage() / 3.3) * (direction == Direction.REVERSE ? -360 : 360);
    }

    /**
     * Filtered angle — smoothed with an exponential moving average to
     * remove analog noise that blows up the derivative term.
     */
    public double getCurrentAngle() {
        double raw = getRawAngle();
        if (!filterInitialized) {
            filteredAngle = raw;
            filterInitialized = true;
            return raw;
        }
        // Wrap-safe delta (handle 359°→1° crossing)
        double delta = raw - filteredAngle;
        if (delta > 180)  delta -= 360;
        if (delta < -180) delta += 360;
        filteredAngle += FILTER_ALPHA * delta;
        // Keep in [0, 360)
        if (filteredAngle < 0)   filteredAngle += 360;
        if (filteredAngle >= 360) filteredAngle -= 360;
        return filteredAngle;
    }

    public boolean isAtTarget() { return isAtTarget(5); }

    public boolean isAtTarget(double tolerance) {
        double err = targetRotation - totalRotation;
        while (err > 180) err -= 360;
        while (err < -180) err += 360;
        return Math.abs(err) < tolerance;
    }

    // ── Reset ──

    public void forceResetTotalRotation() {
        totalRotation = 0;
        previousAngle = getCurrentAngle();
        resetPID();
    }

    public void resetPID() {
        resetIntegral();
        lastError = 0;
        lastDerivative = 0;
        pidTimer.reset();
    }

    public void resetIntegral() { integralSum = 0; }

    // ── Main update — MUST call every loop ──

    public synchronized void update() {
        // Use raw encoder angle directly (0–360°) — no accumulation needed
        double currentAngle = getCurrentAngle();
        totalRotation = currentAngle;  // just store for telemetry/getters
        previousAngle = currentAngle;

        if (!rtp) return;

        double dt = pidTimer.seconds();
        pidTimer.reset();
        if (dt < 0.001 || dt > 1.0) return;

        // Shortest-path error: always goes the short way around the circle
        double error = targetRotation - currentAngle;
        while (error > 180) error -= 360;
        while (error < -180) error += 360;

        // Integral with clamping
        integralSum += error * dt;
        integralSum = Math.max(-maxIntegralSum, Math.min(maxIntegralSum, integralSum));

        // Wind-down near target
        final double INTEGRAL_DEADZONE = 2.0;
        if (Math.abs(error) < INTEGRAL_DEADZONE) {
            integralSum *= 0.95;
        }

        double rawDerivative = (error - lastError) / dt;
        // Clamp derivative to avoid noise spikes
        rawDerivative = Math.max(-MAX_DERIVATIVE, Math.min(MAX_DERIVATIVE, rawDerivative));
        // Low-pass filter the derivative too (smooths out remaining jitter)
        double derivative = 0.4 * rawDerivative + 0.6 * lastDerivative;
        lastError = error;
        lastDerivative = derivative;

        double output = (kP * error) + (kI * integralSum) + (kD * derivative);

        // Deadzone — stop jittering when close
        final double DEADZONE = 3.0;
        if (Math.abs(error) > DEADZONE) {
            double pwr = Math.min(maxPower, Math.abs(output)) * Math.signum(output);
            setPower(pwr);
        } else {
            setPower(0);
        }
    }

    // ── Telemetry helper ──

    @SuppressLint("DefaultLocale")
    public String log() {
        double err = targetRotation - getCurrentAngle();
        while (err > 180) err -= 360;
        while (err < -180) err += 360;
        return String.format(
                "Encoder Volts: %.3f\n" +
                "Raw Angle: %.2f°\n" +
                "Filtered Angle: %.2f°\n" +
                "Target: %.2f°\n" +
                "Error (wrapped): %.2f°\n" +
                "Power: %.3f\n" +
                "PID: P=%.4f I=%.5f D=%.4f\n" +
                "Integral Sum: %.2f\n" +
                "Last Derivative: %.1f",
                servoEncoder.getVoltage(),
                getRawAngle(),
                filteredAngle,
                targetRotation,
                err,
                power,
                kP, kI, kD,
                integralSum,
                lastDerivative
        );
    }
}
