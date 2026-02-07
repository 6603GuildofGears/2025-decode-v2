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
    // These stay constant unless modified via Panels
    public static double KP_TURRET = 0.0004;     // Proportional gain - BASELINE VALUE
    public static double KI_TURRET = 0.01;       // Integral gain - Start at 0, tune up if needed
    public static double KD_TURRET = 0.05;     // Derivative gain - BASELINE VALUE
    
    // ===== CONTROL LIMITS =====
    public static double TURRET_DEADBAND = 0.25;    // Degrees - How close is "good enough"
    public static double MAX_TURRET_SPEED = 0.35;  // Maximum motor power (0-1)
    public static double MAX_INTEGRAL = 0.2;       // Anti-windup limit for integral
    
    // ===== FILTERING =====
    public static double FILTER_ALPHA = 0.8;       // Low-pass filter (0=smooth, 1=responsive)
}
