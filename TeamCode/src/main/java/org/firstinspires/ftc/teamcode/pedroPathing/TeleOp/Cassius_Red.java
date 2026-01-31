package org.firstinspires.ftc.teamcode.pedroPathing.TeleOp;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import static org.firstinspires.ftc.teamcode.pedroPathing.Pipelines.Motor_PipeLine.*;
import static org.firstinspires.ftc.teamcode.pedroPathing.Pipelines.Servo_Pipeline.*;
import static org.firstinspires.ftc.teamcode.pedroPathing.Pipelines.Limelight_Pipeline.*;


@TeleOp(name="Cassius Red", group="TeleOp")
public class Cassius_Red extends LinearOpMode {



    @Override 
    public void runOpMode() throws InterruptedException {

        // pipelines 

        intMotors(this);
        intServos(this);
        initLimelight(this);
        
        // Reset turret encoder to 0 at current position (should be centered manually before init)
        turret.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        turret.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        
        
  

        telemetry.addData("Status", "Hardware initialized");
        telemetry.update();


        // put all initlization code here

        double gear = 1.25; // speed modifier for drive train
        double F1Rest = 0.1; // flicker 1 rest position
        double F2Rest = 0.0875; // flicker 2 rest position
        double F1Shoot = 0.5; // flicker 1 shoot position
        double F2Shoot = 0.5; // flicker 2 shoot position
        
        // Spindexer positions
        double p1 = 0;
        double p2 = 0.425;
        double p3 = 1;
        int currentSpindexerPos = 1; // Track which position (1, 2, or 3)
        boolean dpadPressed = false; // Debounce for dpad
        
        // Flicker timing
        ElapsedTime flickerTimer = new ElapsedTime();
        boolean waitingToFlick = false;

        flicker1.setPosition(F1Rest);
        flicker2.setPosition(F2Rest);
        spindexer.setPosition(p1);

        double rpm = 4000; // target RPM for shooter
      
        // Turret safety limits (FOUND FROM TESTING!)
        int turretMinLimit = -275; // Left limit
        int turretMaxLimit = 630;  // Right limit
        boolean limitsEnabled = true; // Limits are now active

        waitForStart();
        while (opModeIsActive()) {

            // put all TeleOp code here

            //buttons and joysticks

               boolean LStickIn2 = gamepad2.left_stick_button;
                boolean RStickIn2 = gamepad2.right_stick_button;
                boolean LBumper1 = gamepad1.left_bumper;
                boolean RBumper1 = gamepad1.right_bumper;


// talk withe parker abou how to fix controls
                double LStickY =- gamepad1.right_stick_x;       //   inverted
                double LStickX = -gamepad1.left_stick_x;
                double RStickY = -gamepad1.right_stick_y;
                double RStickX = gamepad1.left_stick_y;//inverted

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

            // Intake
            if (LBumper2) {
                intake.setPower(1); // intake in
            } else if (LTrigger2 > 0.1) {
                intake.setPower(-1); // intake out
            } else {
                intake.setPower(0);
            }

            // Flywheel control - simple on/off
            if (RBumper2) {
                flywheel.setVelocity(getTickSpeed(rpm));
            } else {
                flywheel.setVelocity(0);
            }

            // Dpad Right - cycle spindexer position then flick after delay
            if (dpadRight2 && !dpadPressed) {
                currentSpindexerPos++;
                if (currentSpindexerPos > 3) currentSpindexerPos = 1;
                
                // Move spindexer to new position
                if (currentSpindexerPos == 1) spindexer.setPosition(p1);
                else if (currentSpindexerPos == 2) spindexer.setPosition(p2);
                else spindexer.setPosition(p3);
                
                // Start timer for flicking
                flickerTimer.reset();
                waitingToFlick = true;
                dpadPressed = true;
            } else if (!dpadRight2) {
                dpadPressed = false;
            }
            
            // Handle flicker timing
            if (waitingToFlick) {
                if (flickerTimer.milliseconds() < 1500) {
                    // Waiting for spindexer to reach position
                } else if (flickerTimer.milliseconds() < 1800) {
                    // Flick
                    flicker1.setPosition(F1Shoot);
                    flicker2.setPosition(F2Shoot);
                } else {
                    // Reset flickers
                    flicker1.setPosition(F1Rest);
                    flicker2.setPosition(F2Rest);
                    waitingToFlick = false;
                }
            }

            // Hood control - vertical dpad
            double currentHoodPos = hood.getPosition();
            if (dpadUp2) {
                hood.setPosition(Math.min(1.0, currentHoodPos + 0.01)); // Up
            } else if (dpadDown2) {
                hood.setPosition(Math.max(0.0, currentHoodPos - 0.01)); // Down
            }

            // Turret manual control on right stick X (gamepad2)
            int turretPosition = turret.getCurrentPosition();
            double turretPower = RStickX2 * 0.5; // Use right stick X for turret

            // Apply safety limits
            if (limitsEnabled) {
                if (turretPosition <= turretMinLimit && turretPower < 0) {
                    turretPower = 0; // Stop at left limit
                }
                if (turretPosition >= turretMaxLimit && turretPower > 0) {
                    turretPower = 0; // Stop at right limit
                }
            }

            turret.setPower(turretPower);

            // Display telemetry
            telemetry.addData("Turret Position", turret.getCurrentPosition());
            telemetry.addData("Turret Power", String.format("%.2f", turretPower));
            telemetry.addData("Spindexer Pos", currentSpindexerPos);
            telemetry.addData("Flicker1", flicker1.getPosition());
            telemetry.addData("Flicker2", flicker2.getPosition());
            telemetry.addData("Hood", hood.getPosition());
            telemetry.update();
 
        }

    }

    // Helper method to convert RPM to ticks per second
    private double getTickSpeed(double rpm) {
        return rpm * 28 / 60; // 28 ticks per revolution, 60 seconds per minute
    }

 }