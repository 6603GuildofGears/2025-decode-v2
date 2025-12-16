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
        turret = opMode.hardwareMap.get(Servo.class, "turret");


        //Hood servo
        hood = opMode.hardwareMap.get(Servo.class, "hood");
        

        //flicker servo's
        flicker1 = opMode.hardwareMap.get(Servo.class, "flicker1");
        flicker2 = opMode.hardwareMap.get(Servo.class, "flicker2");
        flicker3 = opMode.hardwareMap.get(Servo.class, "flicker3");


        //turret servo direction
        turret.setDirection(Servo.Direction.FORWARD);


        //hood servo direction
        hood.setDirection(Servo.Direction.FORWARD);


        //flicker servo directions
        flicker1.setDirection(Servo.Direction.FORWARD); 
        flicker2.setDirection(Servo.Direction.FORWARD);
        flicker3.setDirection(Servo.Direction.FORWARD);


    }
