package org.firstinspires.ftc.teamcode.pedroPathing.TeleOp;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.pedroPathing.Pipelines.Motor_PipeLine;
import org.firstinspires.ftc.teamcode.pedroPathing.Pipelines.Servo_Pipeline;


@TeleOp(name="V2 TeleOp", group="TeleOp")
public class V2Teleop extends LinearOpMode {



    @Override 
    public void runOpMode() throws InterruptedException {

        // piplines 

        Motor_PipeLine motors = new Motor_PipeLine(this);   
        Servo_Pipeline servos = new Servo_Pipeline(this);

        telemetry.addData("Status", "Hardware initialized");
        telemetry.update();


        // put all initlization code here

        double gear = 1.0; // speed modifier for drive train



        waitForStart();
        while (opModeIsActive()) {

            // put all TeleOp code here

            //buttons and joysticks

               boolean LStickIn2 = gamepad2.left_stick_button;
                boolean RStickIn2 = gamepad2.right_stick_button;
                boolean LBumper1 = gamepad1.left_bumper;
                boolean RBumper1 = gamepad1.right_bumper;

                double LStickY = gamepad1.left_stick_y;
                double LStickX = gamepad1.left_stick_x;
                double RStickY = -gamepad1.right_stick_y;
                double RStickX = -gamepad1.right_stick_x;

                double LTrigger1 = gamepad1.left_trigger; // need to be a value between 0 and 1
                double RTrigger1 = gamepad1.right_trigger; // need to be a value between 0 and 1

                boolean a1 = gamepad1.a;
                boolean b1 = gamepad1.b;
                boolean x1 = gamepad1.x;
                boolean y1 = gamepad1.y;

                boolean a2 = gamepad2.a;
                boolean b2 = gamepad2.b;
                boolean x2 = gamepad2.x;
                boolean y2 = gamepad2.y;

                double LTrigger2 = gamepad2.left_trigger;
                double RTrigger2 = gamepad2.right_trigger;
                boolean LBumper2 = gamepad2.left_bumper;
                boolean RBumper2 = gamepad2.right_bumper;

                double RStickY2 = -gamepad2.right_stick_y;
                double RStickX2 = gamepad2.right_stick_x;
                double LStickY2 = -gamepad2.left_stick_y;
                double LStickX2 = gamepad2.left_stick_x;

                boolean dpadUp1 = gamepad1.dpad_up;
                boolean dpadDown1 = gamepad1.dpad_down;
                boolean dpadRight1 = gamepad1.dpad_right;
                boolean dpadLeft1 = gamepad1.dpad_left;

                boolean dpadUp2 = gamepad2.dpad_up;
                boolean dpadDown2 = gamepad2.dpad_down;
                boolean dpadRight2 = gamepad2.dpad_right;
                boolean dpadLeft2 = gamepad2.dpad_left;


            // Drive code 

            
                if (Math.abs(LStickX) > 0 || Math.abs(LStickY) > 0) {
                    //Orientation angles = imu.getAngularOrientation();
                    double rotation = 0; //Math.toRadians(angles.firstAngle);
                /*
                if (Math.abs(LStickX) < .05 && Math.abs(RStickX) < .05) {
                    SetPower(LStickY, LStickY, LStickY, LStickY);
                }
                else if (Math.abs(LStickY) < .05 && Math.abs(RStickX) < .05) {
                    SetPower(LStickX, -LStickX, -LStickX, LStickX);//+--+
                }
                */

                    double newX = -RStickX * Math.cos(rotation) - -LStickY * Math.sin(rotation); //Angle Difference Identity
                    double newY = LStickY * Math.cos(rotation) - -RStickX * Math.sin(rotation); //Trigonometry

                    double r = Math.hypot(newX, newY);
                    double robotAngle = Math.atan2(newY, newX) - Math.PI / 4;
                    double LeftX = LStickX;
                    double rightX = RStickX;

                    double v1 = r * Math.cos(robotAngle) + LeftX * gear; //lf
                    double v2 = r * Math.sin(robotAngle) + LeftX * gear; //rf
                    double v3 = r * Math.sin(robotAngle) - LeftX * gear; //lb
                    double v4 = r * Math.cos(robotAngle) - LeftX * gear; //rb



                    if (Motor_PipeLine.frontLeft != null) Motor_PipeLine.frontLeft.setPower(v1);
                    if (Motor_PipeLine.frontRight != null) Motor_PipeLine.frontRight.setPower(v2);
                    if (Motor_PipeLine.backLeft != null) Motor_PipeLine.backLeft.setPower(v3);
                    if (Motor_PipeLine.backRight != null) Motor_PipeLine.backRight.setPower(v4);

                } else if (Math.abs(RStickX) > 0) {
                    // Pure turning with right stick X
                    double rightX = RStickX;0

                    double v5 = rightX * gear; //lf
                    double v6 = -rightX * gear; //rf
                    double v7 = rightX * gear; //lb
                    double v8 = -rightX * gear; //rb

                    if (Motor_PipeLine.frontLeft != null) Motor_PipeLine.frontLeft.setPower(v5);
                    if (Motor_PipeLine.frontRight != null) Motor_PipeLine.frontRight.setPower(v6);
                    if (Motor_PipeLine.backLeft != null) Motor_PipeLine.backLeft.setPower(v7);
                    if (Motor_PipeLine.backRight != null) Motor_PipeLine.backRight.setPower(v8);

                } else if (dpadUp1) {
                    if (Motor_PipeLine.frontLeft != null) Motor_PipeLine.frontLeft.setPower(1);
                    if (Motor_PipeLine.frontRight != null) Motor_PipeLine.frontRight.setPower(1);
                    if (Motor_PipeLine.backLeft != null) Motor_PipeLine.backLeft.setPower(1);
                    if (Motor_PipeLine.backRight != null) Motor_PipeLine.backRight.setPower(1); //0.3
                } else if (dpadRight1) {
                    if (Motor_PipeLine.frontLeft != null) Motor_PipeLine.frontLeft.setPower(-1);
                    if (Motor_PipeLine.frontRight != null) Motor_PipeLine.frontRight.setPower(-1);
                    if (Motor_PipeLine.backLeft != null) Motor_PipeLine.backLeft.setPower(-1);
                    if (Motor_PipeLine.backRight != null) Motor_PipeLine.backRight.setPower(-1); //0.5
                } else if (dpadLeft1) {
                    if (Motor_PipeLine.frontLeft != null) Motor_PipeLine.frontLeft.setPower(1);
                    if (Motor_PipeLine.frontRight != null) Motor_PipeLine.frontRight.setPower(1);
                    if (Motor_PipeLine.backLeft != null) Motor_PipeLine.backLeft.setPower(1);
                    if (Motor_PipeLine.backRight != null) Motor_PipeLine.backRight.setPower(1);
                } else if (dpadDown1) {
                    if (Motor_PipeLine.frontLeft != null) Motor_PipeLine.frontLeft.setPower(-1);
                    if (Motor_PipeLine.frontRight != null) Motor_PipeLine.frontRight.setPower(-1);
                    if (Motor_PipeLine.backLeft != null) Motor_PipeLine.backLeft.setPower(-1);
                    if (Motor_PipeLine.backRight != null) Motor_PipeLine.backRight.setPower(-1);


                } else {
                    if (Motor_PipeLine.frontLeft != null) Motor_PipeLine.frontLeft.setPower(0);
                    if (Motor_PipeLine.backLeft != null) Motor_PipeLine.backLeft.setPower(0);
                    if (Motor_PipeLine.frontRight != null) Motor_PipeLine.frontRight.setPower(0);
                    if (Motor_PipeLine.backRight != null) Motor_PipeLine.backRight.setPower(0);
                }



                // AUXILIARY CODE




 
 
        }

    }

 }

