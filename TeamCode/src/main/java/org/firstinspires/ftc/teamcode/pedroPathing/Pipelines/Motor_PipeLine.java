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
        try { frontLeft = opMode.hardwareMap.get(DcMotorEx.class, "frontLeft"); } catch (Exception e) { opMode.telemetry.addData("Warning", "frontLeft motor not found"); }
        try { frontRight = opMode.hardwareMap.get(DcMotorEx.class, "frontRight"); } catch (Exception e) { opMode.telemetry.addData("Warning", "frontRight motor not found"); }
        try { backRight = opMode.hardwareMap.get(DcMotorEx.class, "backRight"); } catch (Exception e) { opMode.telemetry.addData("Warning", "backRight motor not found"); }
        try { backLeft = opMode.hardwareMap.get(DcMotorEx.class, "backLeft"); } catch (Exception e) { opMode.telemetry.addData("Warning", "backLeft motor not found"); }

        // flywheel/intake motors
        try { flywheel = opMode.hardwareMap.get(DcMotorEx.class, "flywheel"); } catch (Exception e) { opMode.telemetry.addData("Warning", "flywheel motor not found"); }
        try { intake = opMode.hardwareMap.get(DcMotorEx.class, "intake"); } catch (Exception e) { opMode.telemetry.addData("Warning", "intake motor not found"); }

        //lift motors
        try { liftLeft = opMode.hardwareMap.get(DcMotorEx.class, "liftLeft"); } catch (Exception e) { opMode.telemetry.addData("Warning", "liftLeft motor not found"); }
        try { liftRight = opMode.hardwareMap.get(DcMotorEx.class, "liftRight"); } catch (Exception e) { opMode.telemetry.addData("Warning", "liftRight motor not found"); }


        //drive motor directions
        if (frontLeft != null) frontLeft.setDirection(DcMotor.Direction.FORWARD); 
        if (frontRight != null) frontRight.setDirection(DcMotor.Direction.FORWARD);
        if (backRight != null) backRight.setDirection(DcMotor.Direction.FORWARD);
        if (backLeft != null) backLeft.setDirection(DcMotor.Direction.FORWARD);

   

        //flywheel and intake motor directions
        if (flywheel != null) flywheel.setDirection(DcMotor.Direction.FORWARD);
        if (intake != null) intake.setDirection(DcMotor.Direction.FORWARD);


       //lift motor directions
        if (liftLeft != null) liftLeft.setDirection(DcMotor.Direction.FORWARD);   
        if (liftRight != null) liftRight.setDirection(DcMotor.Direction.REVERSE);


        //set all motors to zero power behavior to BRAKE
        if (frontLeft != null) frontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        if (frontRight != null) frontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE); 
        if (backRight != null) backRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        if (backLeft != null) backLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);


        //flywheel and intake
        if (flywheel != null) flywheel.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        if (intake != null) intake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);


        //lift motors
        if (liftLeft != null) liftLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE); 
        if (liftRight != null) liftRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

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



}
