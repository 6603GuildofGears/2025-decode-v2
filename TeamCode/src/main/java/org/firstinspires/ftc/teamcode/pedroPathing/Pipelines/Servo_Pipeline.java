package org.firstinspires.ftc.teamcode.pedroPathing.Pipelines;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.CRServo;


public class Servo_Pipeline {

    //turret servo
    static Servo turret;


    //Hood servo
    static Servo hood;
    

    //flicker servo's
    static Servo flicker1;
    static Servo flicker2;
    static Servo flicker3;


    public Servo_Pipeline(OpMode opMode) {

        //turret servo
        try { turret = opMode.hardwareMap.get(Servo.class, "turret"); } catch (Exception e) { opMode.telemetry.addData("Warning", "turret servo not found"); }


        //Hood servo
        try { hood = opMode.hardwareMap.get(Servo.class, "hood"); } catch (Exception e) { opMode.telemetry.addData("Warning", "hood servo not found"); }
        

        //flicker servo's
        try { flicker1 = opMode.hardwareMap.get(Servo.class, "flicker1"); } catch (Exception e) { opMode.telemetry.addData("Warning", "flicker1 servo not found"); }
        try { flicker2 = opMode.hardwareMap.get(Servo.class, "flicker2"); } catch (Exception e) { opMode.telemetry.addData("Warning", "flicker2 servo not found"); }
        try { flicker3 = opMode.hardwareMap.get(Servo.class, "flicker3"); } catch (Exception e) { opMode.telemetry.addData("Warning", "flicker3 servo not found"); }


        //turret servo direction
        if (turret != null) turret.setDirection(Servo.Direction.FORWARD);


        //hood servo direction
        if (hood != null) hood.setDirection(Servo.Direction.FORWARD);


        //flicker servo directions
        if (flicker1 != null) flicker1.setDirection(Servo.Direction.FORWARD); 
        if (flicker2 != null) flicker2.setDirection(Servo.Direction.FORWARD);
        if (flicker3 != null) flicker3.setDirection(Servo.Direction.FORWARD);


    }

}
