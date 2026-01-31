package org.firstinspires.ftc.teamcode.pedroPathing.TeleOp;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.hardware.limelightvision.LLResult;

import static org.firstinspires.ftc.teamcode.pedroPathing.Pipelines.Motor_PipeLine.*;
import static org.firstinspires.ftc.teamcode.pedroPathing.Pipelines.Servo_Pipeline.*;
import static org.firstinspires.ftc.teamcode.pedroPathing.Pipelines.Limelight_Pipeline.*;


@TeleOp(name="Cassius Blue", group="TeleOp")
public class Cassius_Blue extends LinearOpMode {



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
        double F1Rest = 0.1; // flicker 1 rest position
        double F2Rest = 0.0875; // flicker 2 rest position
        double F1Shoot = 0.45; // flicker 1 shoot position
        double F2Shoot = 0.35; // flicker 2 shoot position
        
        ElapsedTime shootTimer = new ElapsedTime();
        int shotCount = 0; // Track which shot we're on (0-3)
        boolean shootingSequence = false; // Is the 3-shot sequence active
        int shootState = 0; // 0=idle, 1=firing, 2=resetting, 3=spinning
        boolean a2Pressed = false; // Track button state to prevent repeat


        flicker1.setPosition(F1Rest);
        flicker2.setPosition(F2Rest);


       



        double rpm = 4000; // target RPM for shooter (NOTE: 'intake' variable is actually the shooter motor)
      
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


            // 3-shot automatic firing sequence
            // Press a2 to start, fires 3 times with spindexer spinning between shots
            // Press RBumper2 to cancel the sequence
            if (a2 && !a2Pressed && !shootingSequence) {
                // Start new shooting sequence
                shootingSequence = true;
                shotCount = 0;
                shootState = 1; // Start with firing
                shootTimer.reset();
                a2Pressed = true;
            } else if (!a2) {
                a2Pressed = false; // Reset button tracking
            }
            
            // Cancel shooting sequence if right bumper 2 is pressed
            if (RBumper2 && shootingSequence) {
                shootingSequence = false;
                shootState = 0;
                spindexer.setPower(0);
                flicker1.setPosition(F1Rest);
                flicker2.setPosition(F2Rest);
            }
            
            // State machine for shooting sequence
            if (shootingSequence) {
                switch (shootState) {
                    case 1: // Firing state
                        flicker1.setPosition(F1Shoot);
                        if (flicker1.getPosition() > 0.075) {
                            flicker2.setPosition(F2Shoot - 0.05);
                        }
                        if (shootTimer.milliseconds() > 400) { // Hold fire position for 400ms (increased)
                            shootState = 2; // Move to reset state
                            shootTimer.reset();
                        }
                        break;
                        
                    case 2: // Reset flickers
                        flicker1.setPosition(F1Rest);
                        flicker2.setPosition(F2Rest);
                        if (shootTimer.milliseconds() > 300) { // Wait 300ms for reset (increased)
                            shotCount++;
                            if (shotCount < 3) {
                                shootState = 3; // Spin spindexer
                                shootTimer.reset();
                                spindexer.setPower(0.5); // Half power for more control
                            } else {
                                // Sequence complete
                                shootingSequence = false;
                                shootState = 0;
                            }
                        }
                        break;
                        
                    case 3: // Spinning spindexer 60 degrees
                        if (shootTimer.milliseconds() > 167) { // ~167ms at 0.5 power for 60 degrees (1/6th rotation)
                            spindexer.setPower(0);
                            shootState = 1; // Go back to firing
                            shootTimer.reset();
                        }
                        break;
                }
            } else {
               
            
                
                if (dpadRight2){
                     spindexer.setPower(1); // spindexer forward
                } else if (dpadLeft2){
                     spindexer.setPower(-1); // spindexer reverse
                } else {
                     spindexer.setPower(0); // spindexer stop
                }
            }

                if (y2 ){
                     hood.setPosition(1); // hood up
                 } else if (x2 ){
                     hood.setPosition(0.25); // hood down
                 } 

          

            // Turret auto-tracking for BLUE alliance (tracks blue goal tag 20 and motif tags 21-23, ignores red goal tag 24)
            int turretPosition = turret.getCurrentPosition();
            double turretPower = 0;
            
            LLResult limelightResult = getLatestResult();
            boolean autoTracking = false;
            
            // Check for valid targets
            if (limelightResult != null && limelightResult.isValid() && 
                limelightResult.getFiducialResults() != null && !limelightResult.getFiducialResults().isEmpty()) {
                
                // Find blue goal only (ignore motif tags and red goal)
                for (int i = 0; i < limelightResult.getFiducialResults().size(); i++) {
                    int tagId = (int) limelightResult.getFiducialResults().get(i).getFiducialId();
                    
                    // Track blue goal (20) ONLY
                    if (tagId == 20) {
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
            
            // Auto-aim only - no manual control

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