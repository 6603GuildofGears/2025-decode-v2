package org.firstinspires.ftc.teamcode.pedroPathing.Pipelines;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;
import com.qualcomm.robotcore.hardware.TouchSensor;

public class Sensor {

    // Touch sensor for magnetic detection
    public static TouchSensor mag;

    // Color sensors for ball detection in spindexer
    public static NormalizedColorSensor ballSensor;
    public static NormalizedColorSensor color2;

    // === Cached sensor 1 values (read once per loop) ===
    private static float s1Red, s1Green, s1Blue, s1Hue, s1Brightness;
    private static boolean s1SeesBall = false;
    private static String s1Color = "NONE";

    // === Cached sensor 2 values (read once per loop) ===
    private static float s2Red, s2Green, s2Blue, s2Hue, s2Brightness;
    private static boolean s2SeesBall = false;
    private static String s2Color = "NONE";

    // === Final cached results ===
    private static boolean cachedBallPresent = false;
    private static String cachedColor = "NONE";
    private static boolean cachedAgree = false;

    public Sensor(OpMode opMode) {
        mag = opMode.hardwareMap.get(TouchSensor.class, "mag");
        ballSensor = opMode.hardwareMap.get(NormalizedColorSensor.class, "ballSensor");
        color2 = opMode.hardwareMap.get(NormalizedColorSensor.class, "color2");
    }

    /**
     * Static initialization method for use in LinearOpMode
     */
    public static void initSensors(OpMode opMode) {
        mag = opMode.hardwareMap.get(TouchSensor.class, "mag");
        ballSensor = opMode.hardwareMap.get(NormalizedColorSensor.class, "ballSensor");
        color2 = opMode.hardwareMap.get(NormalizedColorSensor.class, "color2");
        ballSensor.setGain(4.0f);
        color2.setGain(4.0f);
    }

    /**
     * Convert normalized RGB (0-1) to HSV hue in degrees (0-360).
     */
    public static float rgbToHue(float r, float g, float b) {
        float max = Math.max(r, Math.max(g, b));
        float min = Math.min(r, Math.min(g, b));
        float delta = max - min;

        if (delta < 0.0001f) return 0;

        float hue;
        if (max == r) {
            hue = 60 * (((g - b) / delta) % 6);
        } else if (max == g) {
            hue = 60 * (((b - r) / delta) + 2);
        } else {
            hue = 60 * (((r - g) / delta) + 4);
        }
        if (hue < 0) hue += 360;
        return hue;
    }

    /**
     * Check if RGB values indicate a ball (brightness + hue/channel check).
     */
    private static boolean checkSeesBall(float r, float g, float b) {
        float brightness = Math.max(r, Math.max(g, b));
        if (brightness < 0.08f) return false;

        float hue = rgbToHue(r, g, b);
        if (hue >= 140 && hue < 330) return true;
        if (g > r * 1.2f && g > b * 1.2f && g > 0.05f) return true;
        if (b > r * 0.9f && b > g * 0.9f && b > 0.04f) return true;
        return false;
    }

    /**
     * Classify ball color from cached RGB values.
     */
    private static String classifyColor(float r, float g, float b) {
        float hue = rgbToHue(r, g, b);

        if (hue >= 140 && hue < 195) return "GREEN";
        if (g > r * 1.2f && g > b * 1.2f && g > 0.05f) return "GREEN";
        if (hue >= 195 && hue < 330) return "PURPLE";
        if (b > r * 0.9f && b > g * 0.9f && b > 0.04f) return "PURPLE";
        return "NONE";
    }

    /**
     * CALL ONCE PER LOOP — reads both sensors and caches all results.
     * All other methods use cached values, so they're consistent.
     */
    public static void updateSensors() {
        // --- Read sensor 1 ONCE ---
        NormalizedRGBA rgba1 = ballSensor.getNormalizedColors();
        s1Red = rgba1.red;
        s1Green = rgba1.green;
        s1Blue = rgba1.blue;
        s1Brightness = Math.max(s1Red, Math.max(s1Green, s1Blue));
        s1Hue = rgbToHue(s1Red, s1Green, s1Blue);
        s1SeesBall = checkSeesBall(s1Red, s1Green, s1Blue);
        s1Color = s1SeesBall ? classifyColor(s1Red, s1Green, s1Blue) : "NONE";

        // --- Read sensor 2 ONCE ---
        NormalizedRGBA rgba2 = color2.getNormalizedColors();
        s2Red = rgba2.red;
        s2Green = rgba2.green;
        s2Blue = rgba2.blue;
        s2Brightness = Math.max(s2Red, Math.max(s2Green, s2Blue));
        s2Hue = rgbToHue(s2Red, s2Green, s2Blue);
        s2SeesBall = checkSeesBall(s2Red, s2Green, s2Blue);
        s2Color = s2SeesBall ? classifyColor(s2Red, s2Green, s2Blue) : "NONE";

        // --- Compute final results from cached data ---
        cachedBallPresent = s1SeesBall || s2SeesBall;

        if (s1SeesBall && s2SeesBall) {
            if (s1Color.equals(s2Color)) {
                cachedColor = s1Color;  // both agree
            } else {
                // disagree — use higher brightness
                cachedColor = (s1Brightness >= s2Brightness) ? s1Color : s2Color;
            }
        } else if (s1SeesBall) {
            cachedColor = s1Color;
        } else if (s2SeesBall) {
            cachedColor = s2Color;
        } else {
            cachedColor = "NONE";
        }

        cachedAgree = s1SeesBall || s2SeesBall;
    }

    // === All getters use CACHED values — consistent within one loop ===

    public static boolean isMagPressed() { return mag.isPressed(); }

    public static boolean isBallPresent() { return cachedBallPresent; }

    public static String detectBallColor() { return cachedColor; }

    public static boolean sensorsAgree() { return cachedAgree; }

    public static String getColorSensorDebug() {
        return String.format("S1: H:%.0f B:%.2f R:%.2f G:%.2f B:%.2f → %s",
                s1Hue, s1Brightness, s1Red, s1Green, s1Blue, s1Color);
    }

    public static String getColorSensor2Debug() {
        return String.format("S2: H:%.0f B:%.2f R:%.2f G:%.2f B:%.2f → %s",
                s2Hue, s2Brightness, s2Red, s2Green, s2Blue, s2Color);
    }
}
