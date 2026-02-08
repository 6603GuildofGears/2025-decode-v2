package org.firstinspires.ftc.teamcode.pedroPathing.Pipelines;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;
import com.qualcomm.robotcore.hardware.TouchSensor;

public class Sensor {
    
    // Touch sensor for magnetic detection
    public static TouchSensor mag;

    // Color sensor for ball detection in spindexer
    public static NormalizedColorSensor ballSensor;

    public Sensor(OpMode opMode) {
        mag = opMode.hardwareMap.get(TouchSensor.class, "mag");
        ballSensor = opMode.hardwareMap.get(NormalizedColorSensor.class, "ballSensor");
    }
    
    /**
     * Static initialization method for use in LinearOpMode
     */
    public static void initSensors(OpMode opMode) {
        mag = opMode.hardwareMap.get(TouchSensor.class, "mag");
        ballSensor = opMode.hardwareMap.get(NormalizedColorSensor.class, "ballSensor");
        ballSensor.setGain(4.0f); // higher gain so purple balls read stronger
    }
    
    /**
     * Check if magnetic sensor is pressed/triggered
     * @return true if sensor is pressed
     */
    public static boolean isMagPressed() {
        return mag.isPressed();
    }

    /**
     * Get raw RGBA readings from the ball color sensor
     */
    public static NormalizedRGBA getBallColor() {
        return ballSensor.getNormalizedColors();
    }

    /**
     * Convert normalized RGB (0-1) to HSV hue in degrees (0-360).
     */
    public static float rgbToHue(float r, float g, float b) {
        float max = Math.max(r, Math.max(g, b));
        float min = Math.min(r, Math.min(g, b));
        float delta = max - min;

        if (delta < 0.0001f) return 0; // grey / no color

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
     * Check if a ball is present — true if hue falls in green or purple range.
     */
    public static boolean isBallPresent() {
        NormalizedRGBA c = ballSensor.getNormalizedColors();
        float hue = rgbToHue(c.red, c.green, c.blue);
        return (hue >= 160 && hue < 310);
    }

    /**
     * Detect the ball color using HSV hue only.
     * Green balls ~170°, Purple balls ~220°, split at 195°.
     * @return "GREEN", "PURPLE", or "NONE"
     */
    public static String detectBallColor() {
        NormalizedRGBA c = ballSensor.getNormalizedColors();
        float hue = rgbToHue(c.red, c.green, c.blue);

        // Green: hue 160-195 (~170)
        // Purple: hue 195-310 (~220)
        if (hue >= 160 && hue < 195) return "GREEN";
        if (hue >= 195 && hue < 310) return "PURPLE";

        return "NONE";
    }
}
