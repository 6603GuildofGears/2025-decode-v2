package org.firstinspires.ftc.teamcode.pedroPathing.TeleOp;

import com.bylazar.configurables.annotations.Configurable;

@Configurable
public class AprilTagCentererConfig {
    public static double KP = 0.013;
    public static double FILTER_ALPHA = 0.88;
    public static double DEADBAND_DEG = 0.35;
    public static double MAX_TURRET_SPEED = 0.6;
    public static double SCAN_POWER = 0.075;
    public static double ROTATION_COMP = 0.5; // feedforward from driver rotation
}
