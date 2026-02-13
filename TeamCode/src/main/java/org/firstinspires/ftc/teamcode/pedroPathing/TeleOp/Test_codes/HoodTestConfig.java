package org.firstinspires.ftc.teamcode.pedroPathing.TeleOp;

import com.bylazar.configurables.annotations.Configurable;

/**
 * Configuration class for Hood and Flywheel Testing
 * Adjust these values live via Pedro Pathing Panels
 * 
 * BASELINE VALUES (Always active, even without Panels):
 * These are your tested starting values that will be used by default
 */
@Configurable
public class HoodTestConfig {
    // ===== HOOD POSITION =====
    // Hood servo position for testing (0.0 to 1.0)
    public static double HOOD_POSITION = 0.0;     // Start position - adjust via Panels
    
    // ===== FLYWHEEL RPM =====
    // Target RPM for flywheel motor testing
    public static double TARGET_RPM = 3000.0;     // Starting RPM - adjust via Panels
    
    // ===== FLICKER =====
    // Speed of flicker servo sweep (position units per second)
    // Higher = faster flick. 1.0 means full sweep in 1 second, 5.0 = full sweep in 0.2s
    public static double FLICK_SPEED = 3.0;
    // How long the flicker holds at the shoot position (seconds) before returning
    public static double FLICK_HOLD_TIME = 0.15;

    // ===== TESTING INCREMENTS =====
    // Fine-tune adjustment steps
    public static double HOOD_INCREMENT = 0.01;   // How much hood moves per button press
    public static double RPM_INCREMENT = 50.0;    // How much RPM changes per button press
}
