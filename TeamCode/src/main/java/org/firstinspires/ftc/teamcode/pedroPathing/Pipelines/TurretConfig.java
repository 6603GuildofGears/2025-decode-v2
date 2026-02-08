package org.firstinspires.ftc.teamcode.pedroPathing.TeleOp;

import com.bylazar.configurables.annotations.Configurable;

/**
 * Configuration class for Turret PID Control
 * Adjust these values live via Pedro Pathing Panels
 * 
 * BASELINE VALUES (Always active, even without Panels):
 * These are your tested starting values that will be used by default
 */
@Configurable
public class TurretConfig {
    // ===== PID GAINS =====
    // Tuned via Turret PID Tuner (best err: 0.138°, 133 iterations)
    public static double KP_TURRET = 0.01630;     // Proportional gain
    public static double KI_TURRET = 0.00090;     // Integral gain
    public static double KD_TURRET = 0.00210;     // Derivative gain
    
    // ===== CONTROL LIMITS =====
    public static double TURRET_DEADBAND = 0.25;    // Degrees - How close is "good enough"
    public static double MAX_TURRET_SPEED = 0.35;  // Maximum motor power (0-1)
    public static double MAX_INTEGRAL = 0.2;       // Anti-windup limit for integral
    
    // ===== FILTERING =====
    public static double FILTER_ALPHA = 0.8;       // Low-pass filter (0=smooth, 1=responsive)

    // ===== ODOMETRY-GUIDED TURRET =====
    // Blue goal position in Pedro Pathing coordinates (inches, origin at field corner)
    // From user measurement: (-1.6m, -1.1m) from field center → (9.01", 28.69") in Pedro coords
    public static double BLUE_GOAL_X = 9.01;       // inches (Pedro coords)
    public static double BLUE_GOAL_Y = 28.69;      // inches (Pedro coords)

    // Turret encoder calibration
    // GoBilda 1150 RPM: 145.1 ticks/rev × 6.55 external ratio = 950.4 ticks/turret-rev
    public static double TICKS_PER_TURRET_DEG = 2.64;  // 950.4 / 360
    public static double TURRET_FORWARD_TICKS = 130.5;  // Encoder ticks when turret faces robot-forward

    // Odometry aim speed (how fast turret slews to predicted angle)
    public static double ODO_AIM_POWER = 0.25;     // Max power when seeking via odometry prediction
    public static double ODO_AIM_TOLERANCE = 15.0;  // Degrees — switch to Limelight PID within this window
}
