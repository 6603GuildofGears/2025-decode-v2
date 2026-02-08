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
        ballSensor.setGain(2.0f); // adjust if readings are too dim/bright
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
     * Check if a ball is present in the active spindexer slot.
     * Uses proximity/alpha channel — a ball close to the sensor reflects more light.
     * @param threshold alpha value above which a ball is considered present (tune this)
     */
    public static boolean isBallPresent(double threshold) {
        NormalizedRGBA color = ballSensor.getNormalizedColors();
        return color.alpha > threshold;
    }

    /**
     * Detect the dominant color of the ball currently in the slot.
     * @return "RED", "BLUE", "YELLOW", or "NONE"
     */
    public static String detectBallColor(double presenceThreshold) {
        NormalizedRGBA c = ballSensor.getNormalizedColors();
        if (c.alpha < presenceThreshold) return "NONE";

        float r = c.red, g = c.green, b = c.blue;

        // Yellow shows as high red + high green, low blue
        if (r > b && g > b && g > 0.3f) return "YELLOW";
        if (r > g && r > b) return "RED";
        if (b > r && b > g) return "BLUE";

        return "NONE";
    }
}
