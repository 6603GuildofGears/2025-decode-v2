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
 *   CRServo cr = hardwareMap.crservo.get("spindexer");
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
    private double minPower;
    private Direction direction;

    // Ramp-down: within this many degrees of target, power scales from maxPower → minPower
    private double rampZoneDeg;

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
        kP = 0.003;
        kI = 0.0005;
        kD = 0.00012;
        integralSum = 0.0;
        lastError = 0.0;
        lastDerivative = 0.0;
        maxIntegralSum = 100.0;
        pidTimer = new ElapsedTime();
        pidTimer.reset();

        // Initialize filter with current reading
        filteredAngle = getRawAngle();
        filterInitialized = true;

        maxPower = 0.4;
        minPower = 0.05;
        rampZoneDeg = 60.0;   // start slowing down within 60° of target
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

    public void setMaxPower(double maxPower) { this.maxPower = Math.min(1.0, maxPower); }
    public double getMaxPower() { return maxPower; }
    public void setMinPower(double minPower) { this.minPower = minPower; }
    public double getMinPower() { return minPower; }
    public void setRampZoneDeg(double deg) { this.rampZoneDeg = deg; }
    public double getRampZoneDeg() { return rampZoneDeg; }

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
        // Use RAW angle — the filter lags too much and causes overshoot
        double currentAngle = getRawAngle();
        totalRotation = currentAngle;
        previousAngle = currentAngle;
        // Still update the filter for telemetry comparison
        getCurrentAngle();

        if (!rtp) return;

        // Shortest-path error
        double error = targetRotation - currentAngle;
        while (error > 180) error -= 360;
        while (error < -180) error += 360;

        // PD control: P drives toward target, D brakes to prevent overshoot
        double dt = pidTimer.seconds();
        pidTimer.reset();
        double derivative = (dt > 0) ? (error - lastError) / dt : 0;
        lastError = error;
        lastDerivative = derivative;

        final double DEADZONE = 2.0;
        if (Math.abs(error) > DEADZONE) {
            double pwr = -(kP * error + kD * derivative);
            // Ensure minimum power floor so small errors still move the servo
            if (Math.abs(pwr) < minPower) {
                pwr = minPower * Math.signum(pwr);
            }
            // Clamp to maxPower
            if (pwr > maxPower)  pwr = maxPower;
            if (pwr < -maxPower) pwr = -maxPower;
            setPower(pwr);
        } else {
            setPower(0);
        }
    }

    // ── Telemetry helper ──

    @SuppressLint("DefaultLocale")
    public String log() {
        double raw = getRawAngle();
        double err = targetRotation - raw;
        while (err > 180) err -= 360;
        while (err < -180) err += 360;
        return String.format(
                "Volts: %.3f | Raw: %.1f deg\n" +
                "Target: %.1f deg | Error: %.1f deg\n" +
                "Power: %.3f | kP: %.4f | kD: %.5f\n" +
                "P=%.4f  D=%.4f | minPwr: %.3f",
                servoEncoder.getVoltage(),
                raw,
                targetRotation,
                err,
                power,
                kP,
                kD,
                kP * err,
                lastDerivative,
                minPower
        );
    }
}
