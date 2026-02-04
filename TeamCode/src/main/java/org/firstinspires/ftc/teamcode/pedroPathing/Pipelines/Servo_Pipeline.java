package org.firstinspires.ftc.teamcode.pedroPathing.Pipelines;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.CRServo;


public class Servo_Pipeline {

   


    // //Hood servo
    public static Servo hood;
    

    // //flicker servo's
    public static Servo flicker1;
    public static Servo flicker2;
    

    // spindexer
    public static Servo spindexer;
   

    public Servo_Pipeline(OpMode opMode) {


        // //Hood servo
        hood = opMode.hardwareMap.get(Servo.class, "hood"); 


        // //flicker servo's
        flicker1 = opMode.hardwareMap.get(Servo.class, "flicker1"); 
        flicker2 = opMode.hardwareMap.get(Servo.class, "flicker2");

        // spindexer
        spindexer = opMode.hardwareMap.get(Servo.class, "spindexer");
     
      

        // //hood servo direction
        hood.setDirection(Servo.Direction.FORWARD);


        // //flicker servo directions
         flicker1.setDirection(Servo.Direction.FORWARD); 
         flicker2.setDirection(Servo.Direction.REVERSE);


    // spindexer direction
    spindexer.setDirection(Servo.Direction.FORWARD);


    }

    public static void intServos(OpMode opMode) {



//         //Hood servo
        hood = opMode.hardwareMap.get(Servo.class, "hood"); 
        

//         //flicker servo's
        flicker1 = opMode.hardwareMap.get(Servo.class, "flicker1"); 
        flicker2 = opMode.hardwareMap.get(Servo.class, "flicker2");

        // spindexer
        spindexer = opMode.hardwareMap.get(Servo.class, "spindexer");
//      


//         //hood servo direction
          hood.setDirection(Servo.Direction.FORWARD);


//         //flicker servo directions
         flicker1.setDirection(Servo.Direction.FORWARD); 
         flicker2.setDirection(Servo.Direction.REVERSE);

    // spindexer direction
    spindexer.setDirection(Servo.Direction.FORWARD);
//       
   
    
}

}

