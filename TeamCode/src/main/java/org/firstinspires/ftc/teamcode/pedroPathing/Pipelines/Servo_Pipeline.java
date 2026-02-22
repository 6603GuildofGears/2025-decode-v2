package org.firstinspires.ftc.teamcode.pedroPathing.Pipelines;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.CRServo;


public class Servo_Pipeline {

    // intake servo
    public static CRServo intakeS;

    // //Hood servo
    public static Servo hood;
    

    // //flicker servo
    public static Servo flicker;
    

    // spindexer
    public static Servo spindexer;
   

    public Servo_Pipeline(OpMode opMode) {

   intakeS = opMode.hardwareMap.get(CRServo.class, "intakeS");

        // //Hood servo
        hood = opMode.hardwareMap.get(Servo.class, "hood"); 


        // //flicker servo
        flicker = opMode.hardwareMap.get(Servo.class, "flicker"); 

        // spindexer
        spindexer = opMode.hardwareMap.get(Servo.class, "spindexer");
     
      

        // //hood servo direction
        hood.setDirection(Servo.Direction.FORWARD);


        // //flicker servo direction
         flicker.setDirection(Servo.Direction.FORWARD);


    // spindexer direction
    spindexer.setDirection(Servo.Direction.FORWARD);


    }

    public static void intServos(OpMode opMode) {

// intake
   intakeS = opMode.hardwareMap.get(CRServo.class, "intakeS");

//         //Hood servo
        hood = opMode.hardwareMap.get(Servo.class, "hood"); 
        

//         //flicker servo
        flicker = opMode.hardwareMap.get(Servo.class, "flicker"); 

        // spindexer
        spindexer = opMode.hardwareMap.get(Servo.class, "spindexer");
//      


//         //hood servo direction
          hood.setDirection(Servo.Direction.FORWARD);


//         //flicker servo direction
         flicker.setDirection(Servo.Direction.FORWARD);

    // spindexer direction
    spindexer.setDirection(Servo.Direction.FORWARD);

    intakeS.setDirection(CRServo.Direction.FORWARD);
//       
   
    
}

}

