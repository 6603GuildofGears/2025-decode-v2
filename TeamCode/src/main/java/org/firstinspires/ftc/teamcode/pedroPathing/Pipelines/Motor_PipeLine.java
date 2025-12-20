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

    // flywheel/intake motors
    public static DcMotorEx flywheel;
    public static DcMotorEx intake;

    //lift motors
    public static DcMotorEx liftLeft;
    public static DcMotorEx liftRight;


    public Motor_PipeLine(OpMode opMode) {

        //drive motors
       frontLeft = opMode.hardwareMap.get(DcMotorEx.class, "frontLeft"); 
       frontRight = opMode.hardwareMap.get(DcMotorEx.class, "frontRight");
       backRight = opMode.hardwareMap.get(DcMotorEx.class, "backRight");
       backLeft = opMode.hardwareMap.get(DcMotorEx.class, "backLeft");

        // flywheel/intake motors
        flywheel = opMode.hardwareMap.get(DcMotorEx.class, "flywheel");
        intake = opMode.hardwareMap.get(DcMotorEx.class, "intake");

        //lift motors
        liftLeft = opMode.hardwareMap.get(DcMotorEx.class, "liftLeft");
        liftRight = opMode.hardwareMap.get(DcMotorEx.class, "liftRight");


        //drive motor directions
       frontLeft.setDirection(DcMotor.Direction.FORWARD); 
       frontRight.setDirection(DcMotor.Direction.FORWARD);
       backRight.setDirection(DcMotor.Direction.FORWARD);
       backLeft.setDirection(DcMotor.Direction.FORWARD);

   

        //flywheel and intake motor directions
       flywheel.setDirection(DcMotor.Direction.FORWARD);
       intake.setDirection(DcMotor.Direction.FORWARD);


       //lift motor directions
       liftLeft.setDirection(DcMotor.Direction.FORWARD);   
       liftRight.setDirection(DcMotor.Direction.REVERSE);


        //set all motors to zero power behavior to BRAKE
        frontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE); 
        backRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);


        //flywheel and intake
        flywheel.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        intake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);


        //lift motors
        liftLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE); 
        liftRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

    }

    public static void resetMotors() {
   
        // drive motors
        frontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);


        // flywheel and intake motors
        flywheel.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);  
        intake.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        // lift motors
        liftLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);   
        liftRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);


        // set all motors to run using encoders
        frontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        frontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);


        // flywheel and intake motors
        flywheel.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        intake.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        // lift motors
        liftLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        liftRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

    }

     public static void SetPower(double LFPower, double LBPower, double RFPower, double RBPower) {

        //the names are right but it was buging
        frontLeft.setPower(LFPower);
        backLeft.setPower(LBPower);
        frontRight.setPower(RFPower);
        backRight.setPower(RBPower);
    }

    public static void intMotors(OpMode opMode) {
        int Motor_PipeLine;

        

        //drive motors
       frontLeft = opMode.hardwareMap.get(DcMotorEx.class, "frontLeft"); 
       frontRight = opMode.hardwareMap.get(DcMotorEx.class, "frontRight");
       backRight = opMode.hardwareMap.get(DcMotorEx.class, "backRight");
       backLeft = opMode.hardwareMap.get(DcMotorEx.class, "backLeft");

        // flywheel/intake motors
        flywheel = opMode.hardwareMap.get(DcMotorEx.class, "flywheel");
        intake = opMode.hardwareMap.get(DcMotorEx.class, "intake");

        //lift motors
        liftLeft = opMode.hardwareMap.get(DcMotorEx.class, "liftLeft");
        liftRight = opMode.hardwareMap.get(DcMotorEx.class, "liftRight");


        //drive motor directions
       frontLeft.setDirection(DcMotor.Direction.FORWARD); 
       frontRight.setDirection(DcMotor.Direction.FORWARD);
       backRight.setDirection(DcMotor.Direction.FORWARD);
       backLeft.setDirection(DcMotor.Direction.FORWARD);

   

        //flywheel and intake motor directions
       flywheel.setDirection(DcMotor.Direction.FORWARD);
       intake.setDirection(DcMotor.Direction.FORWARD);


       //lift motor directions
       liftLeft.setDirection(DcMotor.Direction.FORWARD);   
       liftRight.setDirection(DcMotor.Direction.REVERSE);


        //set all motors to zero power behavior to BRAKE
        frontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE); 
        backRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);


        //flywheel and intake
        flywheel.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        intake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);


        //lift motors
        liftLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE); 
        liftRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

    }


}
