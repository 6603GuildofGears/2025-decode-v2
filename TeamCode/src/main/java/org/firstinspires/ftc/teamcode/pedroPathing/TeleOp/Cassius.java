package org.firstinspires.ftc.teamcode.pedroPathing.TeleOp;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import static org.firstinspires.ftc.teamcode.pedroPathing.Pipelines.Motor_PipeLine.*;
import static org.firstinspires.ftc.teamcode.pedroPathing.Pipelines.Servo_Pipeline.*;
import static org.firstinspires.ftc.teamcode.pedroPathing.Pipelines.Limelight_Pipeline.*;


@TeleOp(name="Cassius", group="TeleOp")
public class Cassius extends LinearOpMode {



    @Override 
    public void runOpMode() throws InterruptedException {

        // pipelines 

        intMotors(this);
        intServos(this);
        initLimelight(this);  // Initialize Limelight in AprilTag mode with LEDs off
        
  

        telemetry.addData("Status", "Hardware initialized");
        telemetry.update();


        // put all initlization code here

        double gear = 1.0; // speed modifier for drive train
        double F1Rest = 0.0; // flicker 1 rest position
        double F2Rest = 0.0; // flicker 2 rest position
        double F1Shoot = 0.3; // flicker 1 shoot position
        double F2Shoot = 0.3 ; // flicker 2 shoot position


        flicker1.setPosition(F1Rest);
        flicker2.setPosition(F2Rest);


        double rpm = 3300; // target RPM for shooter (NOTE: 'intake' variable is actually the shooter motor)
      

        waitForStart();
        while (opModeIsActive()) {

            // put all TeleOp code here

            //buttons and joysticks

               boolean LStickIn2 = gamepad2.left_stick_button;
                boolean RStickIn2 = gamepad2.right_stick_button;
                boolean LBumper1 = gamepad1.left_bumper;
                boolean RBumper1 = gamepad1.right_bumper;

                double LStickY = gamepad1.left_stick_y;
                double LStickX = -gamepad1.left_stick_x;
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

               if (Math.abs(LStickX) > 0 || Math.abs(LStickY) > 0 || Math.abs(RStickX) > 0) {
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

                  

                    double r = Math.hypot(LStickX, LStickY);
                    double robotAngle = Math.atan2(LStickY, LStickX) - Math.PI / 4;
                    double rightX = RStickX;

                    double v1 = r * Math.cos(robotAngle) + rightX * gear; //lf
                    double v2 = r * Math.sin(robotAngle) - rightX * gear; //rf
                    double v3 = r * Math.sin(robotAngle) + rightX * gear; //lb
                    double v4 = r * Math.cos(robotAngle) -rightX * gear; //rb



                  SetPower(v1, v3, v2, v4);




                } else if (LBumper1) {
                    SetPower(-gear, gear, gear, -gear);

                } else if (RBumper1) {
                    SetPower(gear, -gear, -gear, gear);

                }  else if (dpadUp1) {
                    SetPower(1 , 1 , 1 , 1 ); //0.3
                } else if (dpadRight1) {
                    SetPower(1, -1, -1, 1); //0.5
                } else if (dpadLeft1) {
                    SetPower(-1, 1, 1, -1);
                } else if (dpadDown1) {
                    SetPower(-1, -1, -1, -1);


                } else {
                    frontLeft.setPower(0);
                    backLeft.setPower(0);
                    frontRight.setPower(0);
                    backRight.setPower(0);
                }





                // AUXILIARY CODE




            // intake 

            if (LBumper2) { // Added a deadzone for the trigger
                intake.setPower(1); // intake in
            } else if (LTrigger2 > 0.1) { // Added a deadzone for the trigger
                intake.setPower(-1); // intake out
            } else {
                intake.setPower(0);
            }


            // flywheel (shooter) control
            if (dpadUp2) {
                flywheel.setVelocity(getTickSpeed(rpm)); // far zone
        
            } else {
                flywheel.setVelocity(0); // idle speed to keep wheel spinning
            }


            // shoot mode - flicker1 fires first, then flicker2 after 0.25 sec
            if (a2) {
                flicker1.setPosition(F1Shoot);
                flicker2.setPosition(F2Shoot);
            }
            if (b2) {
                flicker1.setPosition(F1Rest);
                flicker2.setPosition(F2Rest);
            }

             

                if (dpadRight2){
                     spindexer.setPower(1); // spindexer forward
                } else if (dpadLeft2){
                     spindexer.setPower(-1); // spindexer reverse
                } else {
                     spindexer.setPower(0); // spindexer stop
                }

                if (y2 ){
                     hood.setPosition(1); // hood up
                 } else if (x2 ){
                     hood.setPosition(0.25); // hood down
                 } 

          


            if (LStickX2 > 0.1 ){
                turret.setPower(LStickX2 * 0.75); // half power for finer control
             } else if  (LStickX2 < -0.1 ){
                turret.setPower(LStickX2 * 0.75); // half power for finer control
                }else {
                turret.setPower(0);
             }

            // Display Limelight telemetry
            displayTelemetry(this);
            telemetry.update();
 
        }

    }

    // Helper method to convert RPM to ticks per second
    private double getTickSpeed(double rpm) {
        return rpm * 28 / 60; // 28 ticks per revolution, 60 seconds per minute
    }

 }