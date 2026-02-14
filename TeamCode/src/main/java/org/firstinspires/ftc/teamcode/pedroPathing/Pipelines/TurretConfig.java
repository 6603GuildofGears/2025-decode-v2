package org.firstinspires.ftc.teamcode.pedroPathing.TeleOp;

import com.bylazar.configurables.annotations.Configurable;

/**
 * Turret PID Configuration — live-tunable via Pedro Pathing Panels.
 * Only the essentials: PID gains, deadband, speed, and filter.
 */
@Configurable
public class TurretConfig {
    // ===== PID GAINS =====
    public static double KP_TURRET = 0.01969;     // Proportional gain
    public static double KI_TURRET = 0.00083;     // Integral gain
    public static double KD_TURRET = 0.00210;     // Derivative gain

    // ===== CONTROL =====
    public static double TURRET_DEADBAND = 0.25;   // Degrees — error below this = on-target
    public static double MAX_TURRET_SPEED = 0.35;  // Maximum motor power (0-1)
    public static double SEARCH_SPEED = 0.12;      // Turret power when searching for lost target

    // ===== HEADING LOCK =====
    // Counter-rotates turret when chassis spins. Uses IMU heading (reliable).
    // Units: motor power per degree of heading error.
    // Higher = snappier lock but may oscillate. Lower = smoother but may lag.
    public static double K_HEADING_LOCK = 0.04;

    // ===== FILTERING =====
    public static double FILTER_ALPHA = 0.8;       // Low-pass filter (0=smooth, 1=responsive)

    // ===== ENCODER CONVERSION =====
    // GoBilda 1150 RPM: 145.1 ticks/rev × 6.55 external ratio = 950.4 ticks/turret-rev
    public static double TICKS_PER_DEG = 2.64;     // 950.4 / 360
}
