package org.firstinspires.ftc.teamcode.pedroPathing.TeleOp;

// @Configurable  — hidden from Panels (not actively used)
public class AprilTagCentererConfig {
    public static double KP = 0.013;
    public static double FILTER_ALPHA = 0.88;
    public static double DEADBAND_DEG = 0.35;
    public static double MAX_TURRET_SPEED = 0.6;
    public static double SCAN_POWER = 0.1;
    public static double WRAP_SPEED = 0.6;
    public static double ROTATION_COMP = 0.5; // feedforward from driver rotation
    public static double ROT_COMP_DISABLE_DEG = 8.0; // reduce comp as error grows
    public static double ROT_COMP_MIN_SCALE = 0.2; // minimum comp scale when error is large
}
