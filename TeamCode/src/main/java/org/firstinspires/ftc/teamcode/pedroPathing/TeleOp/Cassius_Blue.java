package org.firstinspires.ftc.teamcode.pedroPathing.TeleOp;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

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
        
        // Simple shooting sequence - only sShot and timer
        ElapsedTime shootTimer = new ElapsedTime();
        int sShot = 0; // 0=idle, 1=shot1, 2=shot2, 3=shot3
        
        // Spindexer oscillation during intake
        ElapsedTime spindexerTimer = new ElapsedTime();
        boolean spindexerAtP1 = true;
        double spindexerOscillateTime = 2000;

        flicker1.setPosition(F1Rest);
        flicker2.setPosition(F2Rest);


       



        double rpm = 4000; // target RPM for shooter (NOTE: 'intake' variable is actually the shooter motor)
      
        // Turret safety limits (FOUND FROM TESTING!)
        int turretMinLimit = -275; // Left limit
        int turretMaxLimit = 630;  // Right limit
        boolean limitsEnabled = true; // Limits are now active

        double p1 = 0;
        double p2 = 0.425;
        double p3 = 1;


        
        // Turret auto-aim constants (PD control for smooth centering)
        double KP_TURRET = 0.018; // Proportional gain (moderate speed)
        double KD_TURRET = 0.35; // Derivative gain (strong damping)
        double TURRET_DEADBAND = 0.5; // Degrees - tight for absolute center
        double MAX_TURRET_SPEED = 0.4; // Maximum turret rotation speed
        double lastTurretError = 0; // For derivative calculation
        double filteredTurretError = 0; // Low-pass filtered error
        double filterAlpha = 0.6; // Filter strength (0.6 = moderate smoothing)
        ElapsedTime turretTimer = new ElapsedTime(); // For time-based derivative

        spindexer.setPosition(p1); // Initialize spindexer to starting position

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




            // intake with spindexer oscillation

            if (RTrigger1 > 0.1) {
                intake.setPower(1); // intake in
                
                double spinmult = 0.02;

                if(spindexer.getPosition() != p1){
                    spindexer.setPosition(spindexer.getPosition() - spinmult);
                } else if (spindexer.getPosition() == p1){
                    spindexer.setPosition(spindexer.getPosition() + spinmult);
                }
                
                // Oscillate spindexer between p1 and p3
                // if (spindexerTimer.milliseconds() > spindexerOscillateTime) {
                //     if (spindexerAtP1) {
                //         spindexer.setPosition(p3);
                //         spindexerAtP1 = false;
                //     } else {
                //         spindexer.setPosition(p1);
                //         spindexerAtP1 = true;
                //     }
                //     spindexerTimer.reset();
                // }
            } else if (LTrigger1 > 0.1) {
                intake.setPower(-1); // intake out
            } else {
                intake.setPower(0);
            }


            // Shooting sequence - fully timer based, no state variables
            // RBumper2 to start, runs through all 3 shots automatically
            if (RBumper2 && sShot == 0) {
                // Start sequence
                sShot = 1;
                shootTimer.reset();
                spindexer.setPosition(p1);
            }
            
            // Run shooting sequence based purely on sShot and timer
            switch(sShot) {
                case 0: // Idle
                    flywheel.setVelocity(0);
                    flicker1.setPosition(F1Rest);
                    flicker2.setPosition(F2Rest);
                    break;
                    
                case 1: // First shot at p1
                    flywheel.setVelocity(getTickSpeed(rpm));
                    if (shootTimer.milliseconds() < 1000) {
                        // Wait for flywheel to spin up
                    } else if (shootTimer.milliseconds() < 1300) {
                        // Fire
                        flicker1.setPosition(F1Shoot);
                        flicker2.setPosition(F2Shoot);
                    } else if (shootTimer.milliseconds() < 2000) {
                        // Reset flickers
                        flicker1.setPosition(F1Rest);
                        flicker2.setPosition(F2Rest);
                    } else if (shootTimer.milliseconds() < 2700) {
                        // Move spindexer to p2
                        spindexer.setPosition(p2);
                    } else {
                        // Next shot
                        sShot = 2;
                        shootTimer.reset();
                    }
                    break;
                    
                case 2: // Second shot at p2
                    flywheel.setVelocity(getTickSpeed(rpm));
                    if (shootTimer.milliseconds() < 300) {
                        // Fire
                        flicker1.setPosition(F1Shoot);
                        flicker2.setPosition(F2Shoot);
                    } else if (shootTimer.milliseconds() < 1750) {
                        // Reset flickers
                        flicker1.setPosition(F1Rest);
                        flicker2.setPosition(F2Rest);
                    } else if (shootTimer.milliseconds() < 2700) {
                        // Move spindexer to p3
                        spindexer.setPosition(p3);
                    } else {
                        // Next shot
                        sShot = 3;
                        shootTimer.reset();
                    }
                    break;
                    
                case 3: // Third shot at p3
                    flywheel.setVelocity(getTickSpeed(rpm));
                    if (shootTimer.milliseconds() < 300) {
                        // Fire
                        flicker1.setPosition(F1Shoot);
                        flicker2.setPosition(F2Shoot);
                    } else if (shootTimer.milliseconds() < 1750) {
                        // Reset flickers
                        flicker1.setPosition(F1Rest);
                        flicker2.setPosition(F2Rest);
                    } else {
                        // Sequence complete
                        sShot = 0;
                    }
                    break;
            }

                // Hood control - slide up/down incrementally
                double currentHoodPos = hood.getPosition();
                if (dpadUp2){
                     hood.setPosition(Math.min(1.0, currentHoodPos + 0.01)); // Slide up
                 } else if (dpadDown2){
                     hood.setPosition(Math.max(0.0, currentHoodPos - 0.01)); // Slide down
                 } 

          

            // Turret control - Manual override or automatic tracking
            int turretPosition = turret.getCurrentPosition();
            double turretPower = 0;
            
            // Manual turret control with dpad left/right on gamepad 2
            if (dpadLeft2) {
                turretPower = -0.5; // Turn left
            } else if (dpadRight2) {
                turretPower = 0.5; // Turn right
            } else if (hasBlueGoal()) {
                // Automatic tracking control (PD control with filtering)
                // Motor direction: RIGHT = POSITIVE, LEFT = NEGATIVE
                double tx = getBlueGoalX(); // Target X offset in degrees
                
                // Low-pass filter to smooth noisy Limelight readings
                filteredTurretError = filterAlpha * tx + (1 - filterAlpha) * filteredTurretError;
                
                if (Math.abs(filteredTurretError) > TURRET_DEADBAND) {
                    // Proportional term: power proportional to error
                    // Positive tx (target right) -> positive power (move right)
                    double pTerm = filteredTurretError * KP_TURRET;
                    
                    // Derivative term: dampens based on rate of change (prevents overshoot)
                    double dt = turretTimer.seconds();
                    double errorChange = (filteredTurretError - lastTurretError) / dt;
                    double dTerm = errorChange * KD_TURRET;
                    turretTimer.reset();
                    
                    // Combined PD control
                    turretPower = pTerm + dTerm;
                    
                    // Clamp to maximum speed
                    if (turretPower > MAX_TURRET_SPEED) turretPower = MAX_TURRET_SPEED;
                    if (turretPower < -MAX_TURRET_SPEED) turretPower = -MAX_TURRET_SPEED;
                    
                    lastTurretError = filteredTurretError;
                } else {
                    lastTurretError = 0; // Reset when centered
                    turretTimer.reset();
                }
            } else {
                lastTurretError = 0; // Reset when no target
                filteredTurretError = 0;
                turretTimer.reset();
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

            // Display telemetry
            telemetry.addData("Turret Position", turret.getCurrentPosition());
            telemetry.addData("Turret Power", String.format("%.2f", turretPower));
            telemetry.addData("Blue Goal Visible", hasBlueGoal() ? "YES" : "NO");
            telemetry.addData("spinPos", spindexer.getPosition());
             if (hasBlueGoal()) {
                telemetry.addData("Target X Error", String.format("%.2f°", getBlueGoalX()));
            }
            telemetry.addData("Spin target: ", sShot);
            telemetry.addData("Shoot timer", shootTimer.milliseconds());
            telemetry.addData("flicker1 pos", flicker1.getPosition());
            telemetry.addData("flicker2 pos", flicker2.getPosition());
            displayTelemetry(this); // Shows Limelight FPS
            telemetry.update();
 
        }

    }

    // Helper method to convert RPM to ticks per second
    private double getTickSpeed(double rpm) {
        return rpm * 28 / 60; // 28 ticks per revolution, 60 seconds per minute
    }

 }