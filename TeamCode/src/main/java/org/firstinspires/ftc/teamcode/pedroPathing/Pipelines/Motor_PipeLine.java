package org.firstinspires.ftc.teamcode.pedroPathing.Pipelines;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;




public class Motor_PipeLine {


    //drive motors
    public static DcMotorEx frontLeft;
    public static DcMotorEx frontRight;
    public static DcMotorEx backRight;
    public static DcMotorEx backLeft;

    // // flywheel/intake motors
     public static DcMotorEx flywheel;
     public static DcMotorEx intake;
     public static DcMotorEx intake2;

     public static DcMotorEx turret;

    // //lift motors
    // public static DcMotorEx lift


    public Motor_PipeLine(OpMode opMode) {

        //drive motors
       frontLeft = opMode.hardwareMap.get(DcMotorEx.class, "frontLeft"); 
       frontRight = opMode.hardwareMap.get(DcMotorEx.class, "backRight");
       backRight = opMode.hardwareMap.get(DcMotorEx.class, "frontRight");
       backLeft = opMode.hardwareMap.get(DcMotorEx.class, "backLeft");

        // // flywheel/intake motors
         flywheel = opMode.hardwareMap.get(DcMotorEx.class, "flywheel");
        intake = opMode.hardwareMap.get(DcMotorEx.class, "intake");
        intake2 = opMode.hardwareMap.get(DcMotorEx.class, "intake2");
        turret = opMode.hardwareMap.get(DcMotorEx.class, "turret");

        // //lift motors
        // lift = opMode.hardwareMap.get(DcMotorEx.class, "lift");


        //drive motor directions
       frontLeft.setDirection(DcMotor.Direction.REVERSE); 
       frontRight.setDirection(DcMotor.Direction.FORWARD);
       backRight.setDirection(DcMotor.Direction.REVERSE);
       backLeft.setDirection(DcMotor.Direction.REVERSE);

   

    //     //flywheel and intake motor directions
    //    flywheel.setDirection(DcMotor.Direction.FORWARD);
       intake.setDirection(DcMotor.Direction.FORWARD);
       intake2.setDirection(DcMotor.Direction.REVERSE);
       flywheel.setDirection(DcMotor.Direction.REVERSE);

        // turret motor direction
        turret.setDirection(DcMotor.Direction.FORWARD);

    //    //lift motor directions
    //    lift.setDirection(DcMotor.Direction.FORWARD);   


        //set all motors to zero power behavior to BRAKE
        frontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE); 
        backRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);


        // //flywheel and intake
         flywheel.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
         intake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
         intake2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);


        // //lift motors
        // lift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE); 


    }

    public static void resetMotors() {
   


        // // flywheel and intake motors
         flywheel.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);  
         intake.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
         intake2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);


        // turret motor
        turret.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        // // lift motors
        // lift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);   

    


        


        // flywheel and intake motors
         flywheel.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
         intake.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
         intake2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        // turret motor
        turret.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        // // lift motors
        // lift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

    }

     public static void SetPower(double LFPower, double LBPower, double RFPower, double RBPower) {

        //the names are right but it was buging
        frontLeft.setPower(LFPower);
        backLeft.setPower(LBPower);
        frontRight.setPower(RFPower);
        backRight.setPower(RBPower);
    }

    public static void intMotors(OpMode opMode) {
     
        //drive motors
       frontLeft = opMode.hardwareMap.get(DcMotorEx.class, "frontLeft"); 
       frontRight = opMode.hardwareMap.get(DcMotorEx.class, "backRight");
       backRight = opMode.hardwareMap.get(DcMotorEx.class, "frontRight");
       backLeft = opMode.hardwareMap.get(DcMotorEx.class, "backLeft");

        // // flywheel/intake motors
         flywheel = opMode.hardwareMap.get(DcMotorEx.class, "flywheel");
         intake = opMode.hardwareMap.get(DcMotorEx.class, "intake");
         intake2 = opMode.hardwareMap.get(DcMotorEx.class, "intake2");

         // turret motor
        turret = opMode.hardwareMap.get(DcMotorEx.class, "turret");

        // //lift motors
        // lift = opMode.hardwareMap.get(DcMotorEx.class, "lift"); 


        //drive motor directions
        frontLeft.setDirection(DcMotor.Direction.FORWARD);     
        frontRight.setDirection(DcMotor.Direction.REVERSE);
        backRight.setDirection(DcMotor.Direction.FORWARD);
        backLeft.setDirection(DcMotor.Direction.FORWARD);

   

    //     //flywheel and intake motor directions
        flywheel.setDirection(DcMotor.Direction.REVERSE);
        intake.setDirection(DcMotor.Direction.REVERSE);
        intake2.setDirection(DcMotor.Direction.REVERSE);

    //     // turret motor direction
        turret.setDirection(DcMotor.Direction.FORWARD);


    //    //lift motor directions
    //    lift.setDirection(DcMotor.Direction.FORWARD);   

        //set all motors to zero power behavior to BRAKE
        frontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE); 
        backRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);


     //flywheel and intake
         flywheel.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
         intake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
         intake2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // turret motor
        turret.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // //lift motors
        // lift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE); 

    }


}
