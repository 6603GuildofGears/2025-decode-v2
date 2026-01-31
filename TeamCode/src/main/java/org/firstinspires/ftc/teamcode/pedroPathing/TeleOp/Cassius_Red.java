package org.firstinspires.ftc.teamcode.pedroPathing.TeleOp;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.hardware.limelightvision.LLResult;

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
        initLimelight(this);  // Initialize Limelight in AprilTag mode with LEDs off
        
        // Reset turret encoder to 0 at current position (should be centered manually before init)
        turret.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        turret.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        
        
  

        telemetry.addData("Status", "Hardware initialized");
        telemetry.update();


        // put all initlization code here

           double gear = 1.25; // speed modifier for drive train
        double F1Rest = 0.0; // flicker 1 rest position
        double F2Rest = 0.0; // flicker 2 rest position      
        double F1Shoot = 0.45; // flicker 1 shoot position
        double F2Shoot = 0.35;

        flicker1.setPosition(F1Rest);
        flicker2.setPosition(F2Rest);


       



        double rpm = 3300; // target RPM for shooter (NOTE: 'intake' variable is actually the shooter motor)
      
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
            if (b2) {
                flywheel.setVelocity(getTickSpeed(rpm)); // far zone
        
            } else {
                flywheel.setVelocity(0); // idle speed to keep wheel spinning
            }


            // shoot mode - flicker1 fires first, then flicker2 after 0.25 sec
              if (a2) {
                flicker1.setPosition(F1Shoot);
               if(flicker1.getPosition() > 0.075){
                flicker2.setPosition(F2Shoot-0.05);
                    }
            } else {
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

          

            // Turret auto-tracking for RED alliance (tracks red goal tag 24 and motif tags 21-23, ignores blue goal tag 20)
            int turretPosition = turret.getCurrentPosition();
            double turretPower = 0;
            
            LLResult limelightResult = getLatestResult();
            boolean autoTracking = false;
            
            // Check for valid targets
            if (limelightResult != null && limelightResult.isValid() && 
                limelightResult.getFiducialResults() != null && !limelightResult.getFiducialResults().isEmpty()) {
                
                // Find red goal only (ignore motif tags and blue goal)
                for (int i = 0; i < limelightResult.getFiducialResults().size(); i++) {
                    int tagId = (int) limelightResult.getFiducialResults().get(i).getFiducialId();
                    
                    // Track red goal (24) ONLY
                    if (tagId == 24) {
                        double tx = limelightResult.getFiducialResults().get(i).getTargetXDegrees();
                        
                        // Small deadzone to prevent micro-adjustments and jitter
                        if (Math.abs(tx) < 2.0) {
                            turretPower = 0;
                            autoTracking = true;
                        } else {
                            // Very smooth proportional control
                            double kP = 0.005; // Lower for smoother, less jittery movement
                            turretPower = tx * kP;
                            
                            // Reduced power limit for smoother control
                            turretPower = Math.max(-0.2, Math.min(0.2, turretPower));
                            autoTracking = true;
                        }
                        break; // Use first valid target
                    }
                }
            }
            
            // Manual control overrides auto-tracking
            if (!autoTracking) {
                if (LStickX2 > 0.1) {
                    turretPower = LStickX2 * 1; // rotate right
                } else if (LStickX2 < -0.1) {
                    turretPower = LStickX2 * 1; // rotate left
                }
            }

            // Apply safety limits to prevent wire damage
            if (limitsEnabled) {
                if (turretPosition <= turretMinLimit && turretPower < 0) {
                    turretPower = 0; // Stop at left limit
                }
                if (turretPosition >= turretMaxLimit && turretPower > 0) {
                    turretPower = 0; // Stop at right limit
                }
            }

            turret.setPower(turretPower);

            // Display Limelight telemetry
            telemetry.addData("turret position", turret.getCurrentPosition());
            telemetry.addData("Auto-Tracking", autoTracking ? "ACTIVE" : "Manual");
            telemetry.addData("flicker1 pos", flicker1.getPosition());
            telemetry.addData("flicker2 pos", flicker2.getPosition());
            displayTelemetry(this);
            telemetry.update();
 
        }

    }

    // Helper method to convert RPM to ticks per second
    private double getTickSpeed(double rpm) {
        return rpm * 28 / 60; // 28 ticks per revolution, 60 seconds per minute
    }

 }