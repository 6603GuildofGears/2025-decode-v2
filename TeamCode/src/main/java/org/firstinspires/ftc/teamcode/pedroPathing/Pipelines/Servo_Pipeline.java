package org.firstinspires.ftc.teamcode.pedroPathing.Pipelines;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.CRServo;


public class Servo_Pipeline {

    // intake servo
    public static CRServo intakeS;

    // //Hood servo
    public static Servo hood;
    

    // //flicker servo
    public static Servo flicker;
    

    // spindexer CRServo (used by RTPAxon)
    public static CRServo spindexer;
    public static AnalogInput spindexerEncoder;
    public static RTPAxon spindexerAxon;
   

    public Servo_Pipeline(OpMode opMode) {

   intakeS = opMode.hardwareMap.get(CRServo.class, "intakeS");

        // //Hood servo
        hood = opMode.hardwareMap.get(Servo.class, "hood"); 


        // //flicker servo
        flicker = opMode.hardwareMap.get(Servo.class, "flicker"); 

        // spindexer RTPAxon
        spindexer = opMode.hardwareMap.get(CRServo.class, "spindexerCR");
        spindexerEncoder = opMode.hardwareMap.get(AnalogInput.class, "spindexerEncoder");
        spindexerAxon = new RTPAxon(spindexer, spindexerEncoder);
        spindexerAxon.setMaxPower(0.4);
        spindexerAxon.setMinPower(0.05);
        spindexerAxon.setKP(0.003);
        spindexerAxon.setKD(0.00012);
        spindexerAxon.setKI(0.0);
      

        // //hood servo direction
        hood.setDirection(Servo.Direction.FORWARD);


        // //flicker servo direction
         flicker.setDirection(Servo.Direction.FORWARD);


    }

    public static void intServos(OpMode opMode) {

// intake
   intakeS = opMode.hardwareMap.get(CRServo.class, "intakeS");

//         //Hood servo
        hood = opMode.hardwareMap.get(Servo.class, "hood"); 
        

//         //flicker servo
        flicker = opMode.hardwareMap.get(Servo.class, "flicker"); 

        // spindexer RTPAxon
        spindexer = opMode.hardwareMap.get(CRServo.class, "spindexerCR");
        spindexerEncoder = opMode.hardwareMap.get(AnalogInput.class, "spindexerEncoder");
        spindexerAxon = new RTPAxon(spindexer, spindexerEncoder);
        spindexerAxon.setMaxPower(0.4);
        spindexerAxon.setMinPower(0.05);
        spindexerAxon.setKP(0.003);
        spindexerAxon.setKD(0.00012);
        spindexerAxon.setKI(0.0);

//         //hood servo direction
          hood.setDirection(Servo.Direction.FORWARD);


//         //flicker servo direction
         flicker.setDirection(Servo.Direction.FORWARD);

    intakeS.setDirection(CRServo.Direction.FORWARD);
//       
   
    
}

}

