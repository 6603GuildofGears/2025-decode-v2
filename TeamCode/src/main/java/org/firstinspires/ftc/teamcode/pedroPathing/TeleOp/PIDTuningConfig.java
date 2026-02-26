package org.firstinspires.ftc.teamcode.pedroPathing.TeleOp;

import com.bylazar.configurables.annotations.Configurable;

/**
 * PID Tuning Configuration — live-tunable via Pedro Pathing Panels.
 *
 * Covers BOTH PID systems:
 *   1. Spindexer (RTPAxon CRServo PD controller)
 *   2. Turret    (DcMotorEx PID via Limelight tx)
 *
 * Open Panels → adjust any value → changes take effect immediately
 * in the PID_Tuner TeleOp (no restart needed).
 */
@Configurable
public class PIDTuningConfig {

    // ═══════════════════════════════════════════════
    //  SPINDEXER  (RTPAxon — CRServo + AnalogInput)
    // ═══════════════════════════════════════════════

    // --- Gains ---
    public static double SPINDEXER_KP       = 0.003;    // Proportional gain
    public static double SPINDEXER_KI       = 0.0;      // Integral gain (keep 0 for CRServo)
    public static double SPINDEXER_KD       = 0.00012;  // Derivative gain

    // --- Power limits ---
    public static double SPINDEXER_MAX_PWR  = 0.4;      // Max output power (0–1)
    public static double SPINDEXER_MIN_PWR  = 0.05;     // Min power floor (overcomes static friction)

    // --- Behaviour ---
    public static double SPINDEXER_DEADZONE = 2.0;      // Degrees inside which output = 0
    public static double SPINDEXER_TARGET   = 60.0;     // Target angle — drag slider to command a position

    // ═══════════════════════════════════════════════
    //  TURRET  (DcMotorEx PID via Limelight tx)
    // ═══════════════════════════════════════════════

    // --- Gains ---
    public static double TURRET_KP          = 0.015;    // Proportional gain
    public static double TURRET_KI          = 0.0001;   // Integral gain
    public static double TURRET_KD          = 0.002;    // Derivative gain

    // --- Power limits ---
    public static double TURRET_MAX_PWR     = 0.8;      // Max turret motor power
    public static double TURRET_MIN_PWR     = 0.05;     // Min power floor

    // --- Behaviour ---
    public static double TURRET_DEADBAND    = 1.5;      // Degrees — error below this = on-target
    public static boolean TURRET_INVERT     = false;     // Flip motor direction

    // --- Mode selector (change in Panels) ---
    // 0 = Spindexer only,  1 = Turret only,  2 = Both
    public static double ACTIVE_SYSTEM      = 0;
}
