package org.firstinspires.ftc.teamcode.pedroPathing.TeleOp;

// @Configurable  — hidden from Panels (not actively used)
public class TurretTunerConfig {
    public static boolean AUTO_TUNE_ENABLED = false;
    public static double RESPONSE_TIME_SECONDS = 1.0; // target response time
    public static double SPEED_PRIORITY = 0.6; // 0=accuracy, 1=speed

    public static double KP_MAX = 0.2;
    public static double KI_MAX = 0.01;
    public static double KD_MAX = 0.05;
    public static double ALPHA_MIN = 0.2;
    public static double ALPHA_MAX = 0.95;
    public static double DEADBAND_MIN = 0.2;
    public static double DEADBAND_MAX = 2.0;
}
