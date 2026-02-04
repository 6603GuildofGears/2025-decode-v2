package org.firstinspires.ftc.teamcode.pedroPathing.Pipelines;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.TouchSensor;

public class Sensor {
    
    // Touch sensor for magnetic detection
    public static TouchSensor mag;

    public Sensor(OpMode opMode) {
        mag = opMode.hardwareMap.get(TouchSensor.class, "mag");
    }
    
    /**
     * Static initialization method for use in LinearOpMode
     */
    public static void initSensors(OpMode opMode) {
        mag = opMode.hardwareMap.get(TouchSensor.class, "mag");
    }
    
    /**
     * Check if magnetic sensor is pressed/triggered
     * @return true if sensor is pressed
     */
    public static boolean isMagPressed() {
        return mag.isPressed();
    }
}
