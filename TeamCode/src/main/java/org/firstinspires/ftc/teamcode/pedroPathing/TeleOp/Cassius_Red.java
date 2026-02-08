package org.firstinspires.ftc.teamcode.pedroPathing.TeleOp;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import static org.firstinspires.ftc.teamcode.pedroPathing.Pipelines.Motor_PipeLine.*;
import static org.firstinspires.ftc.teamcode.pedroPathing.Pipelines.Servo_Pipeline.*;
import static org.firstinspires.ftc.teamcode.pedroPathing.Pipelines.Limelight_Pipeline.*;
import static org.firstinspires.ftc.teamcode.pedroPathing.Pipelines.Sensor.*;
import static org.firstinspires.ftc.teamcode.pedroPathing.TeleOp.TurretConfig.*;


@TeleOp(name="Cassius Red", group="TeleOp")
public class Cassius_Red extends LinearOpMode {



    @Override 
    public void runOpMode() throws InterruptedException {

        // pipelines 

        intMotors(this);
        intServos(this);
        initLimelight(this);
        initSensors(this);
        
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
        double servoTolerance = 0.05; // How close servo needs to be to target (tolerance)
        
        // Simple shooting sequence - only sShot and timer
        ElapsedTime shootTimer = new ElapsedTime();
        int sShot = 0; // 0=idle, 1=shot1, 2=shot2, 3=shot3
        
        // Spindexer rotation during intake
        boolean spindexerForward = true; // Direction of rotation
        
        // Turret tracking toggle
        boolean autoTrackingEnabled = true; // Default to auto-tracking
        boolean y2Pressed = false; // Debounce for Y button
        flicker1.setPosition(F1Rest);
        flicker2.setPosition(F2Rest);

        double rpm = 3000; // target RPM for shooter (NOTE: 'intake' variable is actually the shooter motor)
      
        // Turret gear ratio - Motor has 20 teeth, Turret gear has 131 teeth
        double turretGearRatio = 131.0 / 20.0; // = 6.55:1 (motor turns 6.55x for each turret rotation)
        
        // Turret safety limits (FOUND FROM TESTING!)
        int turretMinLimit = -275; // Left limit
        int turretMaxLimit = 630;  // Right limit
        boolean limitsEnabled = true; // Limits are now active
        
        // Mag sensor calibration position
        int magSensorPosition = -149; // Turret position when mag sensor triggers (-145 to -153 range)
        boolean lastMagState = false; // Track mag sensor state changes

        double p1 = 0;
        double p2 = 0.5;
        double p3 = 1;

        // Turret PID values are now in TurretConfig.java for live tuning via Pedro Pathing Panels
        
        // Turret tracking state variables
        double lastTurretError = 0;
        double integratedError = 0; // Accumulated error for integral term
        double filteredTurretError = 0;
        ElapsedTime turretTimer = new ElapsedTime();

        spindexer.setPosition(p1); // Initialize spindexer to starting position

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

            // Intake with spindexer slow rotation
            if (LBumper2) {
                intake.setPower(1); // intake in
                
                // Continuously rotate spindexer
                double currentPos = spindexer.getPosition();
                currentPos += 0.005; // Increment for slow rotation
                if (currentPos == 1) currentPos = 0.0; // Wrap back to start
                spindexer.setPosition(currentPos);
            } else if (LTrigger2 > 0.1) {
                intake.setPower(-1); // intake out
            } else {
                intake.setPower(0);
            }
            
            // Spindexer oscillation during intake
            if (RTrigger2 > 0.1) {
                double currentPos = spindexer.getPosition();

                if(spindexerForward){
                spindexer.setPosition(currentPos + 0.005);
                    if(currentPos >= 0.995) {
                        spindexerForward = false; // Reverse direction
                    }
                } else {
                    spindexer.setPosition(currentPos - 0.005);
                    if(currentPos <= 0.005) {
                        spindexerForward = true; // Reverse direction
                    }
                }
            }

            // Shooting sequence - fully timer based, no state variables
            // RBumper2 to start, runs through all 3 shots automatically
            // LBumper2 as KILL SWITCH to stop sequence immediately
            if (LBumper2 && sShot != 0) {
                // KILL SWITCH - Emergency stop of shooting sequence
                sShot = 0;
                flywheel.setVelocity(0);
                flicker1.setPosition(F1Rest);
                flicker2.setPosition(F2Rest);
                shootTimer.reset();
            } else if (RBumper2 && sShot == 0) {
                // Start sequence
                sShot = 1;
                shootTimer.reset();
                spindexer.setPosition(p1);
            }
            
            // Run shooting sequence based purely on sShot and timer
            // FAIL-SAFE: Each case has a maximum timeout to prevent jamming from breaking the sequence
            switch(sShot) {
                case 0: // Idle
                    flywheel.setVelocity(0);
                    flicker1.setPosition(F1Rest);
                    flicker2.setPosition(F2Rest);
                    break;
                    
                case 1: // First shot at p1
                    flywheel.setVelocity(getTickSpeed(rpm));
                    if (shootTimer.milliseconds() < 1500) {
                        // Wait for flywheel to spin up
                    } else if (shootTimer.milliseconds() < 1900) {
                        // Fire - command servos
                        flicker1.setPosition(F1Shoot);
                        flicker2.setPosition(F2Shoot);
                    } else if (shootTimer.milliseconds() < 2100) {
                        // Check if flickers reached target OR timeout
                        boolean f1Ready = Math.abs(flicker1.getPosition() - F1Shoot) < servoTolerance;
                        boolean f2Ready = Math.abs(flicker2.getPosition() - F2Shoot) < servoTolerance;
                        
                        if (f1Ready && f2Ready || shootTimer.milliseconds() > 2000) {
                            // Reset flickers (either reached target or timed out)
                            flicker1.setPosition(F1Rest);
                            flicker2.setPosition(F2Rest);
                        }
                    } else if (shootTimer.milliseconds() < 3200) {
                        // Move spindexer to p2
                        spindexer.setPosition(p2);
                    } else if (shootTimer.milliseconds() < 4000) {
                        // Wait for spindexer to finish
                    } else {
                        // Next shot OR timeout fail-safe (4 seconds max)
                        sShot = 2;
                        shootTimer.reset();
                    }
                    break;
                    
                case 2: // Second shot at p2
                    flywheel.setVelocity(getTickSpeed(rpm));
                    if (shootTimer.milliseconds() < 600) {
                        // Fire - command servos
                        flicker1.setPosition(F1Shoot);
                        flicker2.setPosition(F2Shoot);
                    } else if (shootTimer.milliseconds() < 1000) {
                        // Check if flickers reached target OR timeout
                        boolean f1Ready = Math.abs(flicker1.getPosition() - F1Shoot) < servoTolerance;
                        boolean f2Ready = Math.abs(flicker2.getPosition() - F2Shoot) < servoTolerance;
                        
                        if (f1Ready && f2Ready || shootTimer.milliseconds() > 900) {
                            // Reset flickers (either reached target or timed out)
                            flicker1.setPosition(F1Rest);
                            flicker2.setPosition(F2Rest);
                        }
                    } else if (shootTimer.milliseconds() < 2500) {
                        // Move spindexer to p3
                        spindexer.setPosition(p3);
                    } else if (shootTimer.milliseconds() < 3500) {
                        // Wait for spindexer to finish
                    } else {
                        // Next shot OR timeout fail-safe (3.5 seconds max)
                        sShot = 3;
                        shootTimer.reset();
                    }
                    break;
                    
                case 3: // Third shot at p3
                    flywheel.setVelocity(getTickSpeed(rpm));
                    if (shootTimer.milliseconds() < 300) {
                        // Fire - command servos
                        flicker1.setPosition(F1Shoot);
                        flicker2.setPosition(F2Shoot);
                    } else if (shootTimer.milliseconds() < 1000) {
                        // Check if flickers reached target OR timeout
                        boolean f1Ready = Math.abs(flicker1.getPosition() - F1Shoot) < servoTolerance;
                        boolean f2Ready = Math.abs(flicker2.getPosition() - F2Shoot) < servoTolerance;
                        
                        if (f1Ready && f2Ready || shootTimer.milliseconds() > 900) {
                            // Reset flickers (either reached target or timed out)
                            flicker1.setPosition(F1Rest);
                            flicker2.setPosition(F2Rest);
                        }
                    } else if (shootTimer.milliseconds() < 2000) {
                        // Extra time buffer for final shot
                    } else {
                        // Sequence complete OR timeout fail-safe (2 seconds max)
                        sShot = 0;
                    }
                    break;
            }

            // Hood control - slide up/down incrementally
            double currentHoodPos = hood.getPosition();
            if (dpadUp2) {
                hood.setPosition(Math.min(1.0, currentHoodPos + 0.01)); // Slide up
            } else if (dpadDown2) {
                hood.setPosition(Math.max(0.0, currentHoodPos - 0.01)); // Slide down
            }

            // Toggle turret auto-tracking with Y button (gamepad2)
            if (y2 && !y2Pressed) {
                autoTrackingEnabled = !autoTrackingEnabled;
                // Reset tracking state when starting tracking
                lastTurretError = 0;
                integratedError = 0; // Reset integral on toggle
                filteredTurretError = 0;
                turretTimer.reset();
                y2Pressed = true;
            } else if (!y2) {
                y2Pressed = false;
            }

            // Mag sensor calibration - Reset turret encoder if sensor triggered
            boolean currentMagState = isMagPressed();
            if (currentMagState && !lastMagState) {
                // Mag sensor just triggered - check if turret position needs correction
                int currentPos = turret.getCurrentPosition();
                int positionError = Math.abs(currentPos - magSensorPosition);
                
                if (positionError > 5) {
                    // Position is off by more than 5 ticks - recalibrate
                    turret.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                    turret.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                    // Manually set the position by moving to the calibration point
                    int offsetNeeded = magSensorPosition;
                    // Note: Since we just reset to 0, we need to track this offset
                    telemetry.addData("TURRET CALIBRATED", "Reset to %d", magSensorPosition);
                }
            }
            lastMagState = currentMagState;
            
            // Turret control - Manual override or automatic tracking
            int turretPosition = turret.getCurrentPosition();
            double turretPower = 0;
            
            // Manual turret control with dpad left/right on gamepad 2
            // if (dpadLeft2) {
            //     turretPower = -0.5; // Turn left
            //     // Reset tracking when manual override
            //     lastTurretError = 0;
            //     filteredTurretError = 0;
            // } else if (dpadRight2) {
            //     turretPower = 0.5; // Turn right
            //     // Reset tracking when manual override
            //     lastTurretError = 0;
            //     filteredTurretError = 0;
            // } else 
            if (autoTrackingEnabled && hasRedGoal()) {
                // Automatic tracking control (PID control with filtering)
                // Motor direction: RIGHT = POSITIVE, LEFT = NEGATIVE
                double tx = getRedGoalX(); // Target X offset in degrees
                
                // Initialize filter on first reading to avoid lag
                if (filteredTurretError == 0 && lastTurretError == 0) {
                    filteredTurretError = tx;
                } else {
                    // Low-pass filter to smooth noisy Limelight readings
                    filteredTurretError = FILTER_ALPHA * tx + (1 - FILTER_ALPHA) * filteredTurretError;
                }
                
                if (Math.abs(filteredTurretError) > TURRET_DEADBAND) {
                    // Apply gear ratio compensation - motor must turn 6.55x more than turret angle
                    double compensatedError = filteredTurretError * turretGearRatio;
                    
                    // Proportional term: power proportional to error (with gear compensation)
                    double pTerm = compensatedError * KP_TURRET;
                    
                    // Derivative term: dampens based on rate of change
                    double dt = turretTimer.seconds();
                    double dTerm = 0;
                    
                    // Only calculate derivative if we have a valid time delta
                    if (dt > 0.01 && dt < 1.0) {
                        double errorChange = (filteredTurretError - lastTurretError) / dt;
                        // Clamp derivative to prevent spikes
                      //  if (Math.abs(errorChange) < 100) {
                            dTerm = errorChange * KD_TURRET;
                      //  }
                    }
                    
                    // Integral term: accumulate error over time (with anti-windup)
                    integratedError += filteredTurretError * dt;
                    integratedError = Math.max(-MAX_INTEGRAL, Math.min(MAX_INTEGRAL, integratedError));
                    double iTerm = integratedError * KI_TURRET;
                    
                    // Combined PID control
                    turretPower = pTerm + iTerm + dTerm;
                    
                    // Clamp to maximum speed
                    turretPower = Math.max(-MAX_TURRET_SPEED, Math.min(MAX_TURRET_SPEED, turretPower));
                    
                    lastTurretError = filteredTurretError;
                    turretTimer.reset();
                } else {
                    // Within deadband - centered
                    turretPower = 0;
                    lastTurretError = filteredTurretError;
                }
            } else {
                // No tracking or no target
                lastTurretError = 0;
                integratedError = 0; // Reset integral when target lost
                filteredTurretError = 0;
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

            // ===== LIMELIGHT STATUS SECTION =====
            telemetry.addData("=== LIMELIGHT STATUS ===", "");
            telemetry.addData("LL Connected", hasTarget() ? "YES" : "CHECKING...");
            telemetry.addData("Red Goal Visible", hasRedGoal() ? "YES" : "NO");
            if (hasRedGoal()) {
                telemetry.addData("Target X Error", String.format("%.2f°", getRedGoalX()));
            }
            
            telemetry.addData("=== TURRET ===", "");
            telemetry.addData("Turret Position", turret.getCurrentPosition());
            telemetry.addData("Turret Power", String.format("%.2f", turretPower));
            telemetry.addData("Auto Tracking", autoTrackingEnabled ? "ON" : "OFF");
            telemetry.addData("Filtered Error", String.format("%.2f°", filteredTurretError));
            telemetry.addData("Integrated Error", String.format("%.3f", integratedError));
            telemetry.addData("--- PID TUNING (Panels) ---", "");
            telemetry.addData("KP", String.format("%.3f", KP_TURRET));
            telemetry.addData("KI", String.format("%.3f", KI_TURRET));
            telemetry.addData("KD", String.format("%.3f", KD_TURRET));
            
            telemetry.addData("=== SHOOTER ===", "");
            telemetry.addData("Shot State", sShot == 0 ? "IDLE" : "Shot " + sShot);
            telemetry.addData("Shoot Timer", String.format("%.0fms", shootTimer.milliseconds()));
            telemetry.addData("Flywheel Velocity", String.format("%.0f", flywheel.getVelocity()));
            
            telemetry.addData("=== SERVOS ===", "");
            telemetry.addData("Spindexer Pos", String.format("%.2f", spindexer.getPosition()));
            telemetry.addData("Flicker1", String.format("%.2f", flicker1.getPosition()));
            telemetry.addData("Flicker2", String.format("%.2f", flicker2.getPosition()));
            
            displayTelemetry(this); // Shows Limelight FPS and additional info
            telemetry.update();
 
        }

    }

    // Helper method to convert RPM to ticks per second
    private double getTickSpeed(double rpm) {
        return rpm * 28 / 60; // 28 ticks per revolution, 60 seconds per minute
    }

 }