package org.firstinspires.ftc.teamcode.pedroPathing.TeleOp;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.hardware.limelightvision.LLResult;


import static org.firstinspires.ftc.teamcode.pedroPathing.Pipelines.Motor_PipeLine.*;
import static org.firstinspires.ftc.teamcode.pedroPathing.Pipelines.Servo_Pipeline.*;
import static org.firstinspires.ftc.teamcode.pedroPathing.Pipelines.Limelight_Pipeline.*;
import static org.firstinspires.ftc.teamcode.pedroPathing.Pipelines.Sensor.*;
import static org.firstinspires.ftc.teamcode.pedroPathing.TeleOp.TurretConfig.*;



@Disabled
@TeleOp(name="Cassius Blue Simple", group="TeleOp")
public class Cassius_Blue_Simple extends LinearOpMode {



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

        double gear = 1.25; // speed modifier for drive train
        double F1Rest = 0.1;
        double F2Rest = 0.0875;
        double F1Shoot = 0.5;
        double F2Shoot = 0.5;
        double servoTolerance = 0.05;
        
        ElapsedTime shootTimer = new ElapsedTime();
        int sShot = 0;
        
        boolean spindexerForward = true;
        double spindexerIncrementSpeed = 0.075;
        
        boolean autoTrackingEnabled = true;
        boolean y2Pressed = false;
        flicker1.setPosition(F1Rest);
        flicker2.setPosition(F2Rest);

        double baseRpm = 3000;
        double targetRpm = baseRpm;
        double targetHoodPos = hood.getPosition();
        double cameraHeight = 11.125;
        double cameraMountAngle = 22.85; // calibrated
        double targetHeight = 29.5;
      
        double turretGearRatio = 131.0 / 20.0;
        int turretMinLimit = -275;
        int turretMaxLimit = 630;
        boolean limitsEnabled = true;
        int magSensorPosition = -149;
        boolean lastMagState = false;

        double p1 = 0;
        double p2 = 0.375;
        double p3 = .75;

        int startSpindexer = 0;
        double spindexerTargetPos = p3;

        double lastTurretError = 0;
        double integratedError = 0;
        double filteredTurretError = 0;
        ElapsedTime turretTimer = new ElapsedTime();

        boolean sfpo = true;

        waitForStart();
        while (opModeIsActive()) {
               boolean LStickIn2 = gamepad2.left_stick_button;
                boolean RStickIn2 = gamepad2.right_stick_button;
                boolean LBumper1 = gamepad1.left_bumper;
                boolean RBumper1 = gamepad1.right_bumper;

                // ===== SIMPLE DRIVE CONTROLS =====
                // Left stick Y = forward/backward (inverted because gamepad Y is inverted)
                // Left stick X = strafe left/right
                // Right stick X = rotate left/right
                double drive  = gamepad1.left_stick_y;   // Forward = positive (inverted)
                double strafe = -gamepad1.left_stick_x;   // Right = positive (inverted)
                double turn   = -gamepad1.right_stick_x;  // Clockwise = positive (inverted)

                double LTrigger1 = gamepad1.left_trigger;
                double RTrigger1 = gamepad1.right_trigger;

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

            // ===== SUPER SIMPLE MECANUM DRIVE =====
            // LF and RB are diagonal partners (same power for strafe)
            // RF and LB are diagonal partners (same power for strafe)

            if (Math.abs(drive) > 0.05 || Math.abs(strafe) > 0.05 || Math.abs(turn) > 0.05) {

                double leftFrontPower  = (drive + strafe + turn) * gear;
                double leftBackPower   = (drive - strafe + turn) * gear;
                double rightFrontPower = (drive - strafe - turn) * gear;
                double rightBackPower  = (drive + strafe - turn) * gear;

                // Normalize so no motor exceeds 1.0
                double max = Math.max(Math.abs(leftFrontPower),
                             Math.max(Math.abs(leftBackPower),
                             Math.max(Math.abs(rightFrontPower),
                                      Math.abs(rightBackPower))));
                if (max > 1.0) {
                    leftFrontPower  /= max;
                    leftBackPower   /= max;
                    rightFrontPower /= max;
                    rightBackPower  /= max;
                }

                frontLeft.setPower(leftFrontPower);
                backLeft.setPower(leftBackPower);
                frontRight.setPower(rightFrontPower);
                backRight.setPower(rightBackPower);

            } else if (LBumper1) {
                    SetPower(gear, -gear, gear, -gear);

                } else if (LTrigger1 > 0.25) {
                    SetPower(gear, -gear, gear, -gear);

                }  else if (dpadUp1) {
                    SetPower(1 , 1 , 1 , 1 );
                } else if (dpadRight1) {
                    SetPower(1, -1, -1, 1);
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

            telemetry.addData("spindexer pos", spindexer.getPosition());
            if(dpadRight2){
                double CPoS = spindexer.getPosition();
                spindexer.setPosition(CPoS + 0.005);;
            } else if (dpadLeft2){
                double CPoS = spindexer.getPosition();
                spindexer.setPosition(CPoS - 0.005);;
            }

            // intake with spindexer slow rotation
            if (RTrigger1 > 0.1) {
                intake.setPower(0.75);
            } else if (RBumper1) {
                intake.setPower(-1);
            } else {
                intake.setPower(0);
            }

            // Limelight-based shooter tuning
            LLResult limelightResult = getLatestResult();
            double distanceInches = 0.0;
            boolean hasDistance = false;
            if (limelightResult != null && limelightResult.isValid() &&
                limelightResult.getFiducialResults() != null && !limelightResult.getFiducialResults().isEmpty()) {
                double ty = limelightResult.getFiducialResults().get(0).getTargetYDegrees();
                double totalAngle = cameraMountAngle + ty;
                double heightDifference = targetHeight - cameraHeight;
                if (Math.abs(totalAngle) > 0.5 && Math.abs(totalAngle) < 89.5) {
                    distanceInches = heightDifference / Math.tan(Math.toRadians(totalAngle));
                    hasDistance = true;
                }
            }
            if (hasDistance && (sShot != 0 || RBumper2) && !(dpadUp2 || dpadDown2)) {
                ShooterLookup.Result tuned = ShooterLookup.lookup(distanceInches);
                targetRpm = tuned.rpm;
                targetHoodPos = tuned.hoodPos;
                hood.setPosition(targetHoodPos);
            } else if (!hasDistance) {
                targetRpm = baseRpm;
            }

            // Shooting sequence
            if (LBumper2 && sShot != 0) {
                sShot = 0;
                flywheel.setVelocity(0);
                flicker1.setPosition(F1Rest);
                flicker2.setPosition(F2Rest);
                shootTimer.reset();
            } 
            if (RBumper2 && sShot == 0) {
                sShot = 1;
                shootTimer.reset();
                
                if(spindexer.getPosition() < p2){
                    spindexerTargetPos = p1;
                    spindexer.setPosition(p1);
                    sfpo = true;
                } else {
                    spindexerTargetPos = p3;
                    spindexer.setPosition(p3);
                    sfpo = false;
                }
            }
            
            if(sfpo){
                switch(sShot) {
                    case 0:
                        flywheel.setVelocity(0);
                        flicker1.setPosition(F1Rest);
                        flicker2.setPosition(F2Rest);
                        break;
                    case 1:
                        flywheel.setVelocity(getTickSpeed(targetRpm));
                        if (shootTimer.milliseconds() < 1500) {
                        } else if (shootTimer.milliseconds() < 1900) {
                            flicker1.setPosition(F1Shoot);
                            flicker2.setPosition(F2Shoot);
                        } else if (shootTimer.milliseconds() < 2100) {
                            boolean f1Ready = Math.abs(flicker1.getPosition() - F1Shoot) < servoTolerance;
                            boolean f2Ready = Math.abs(flicker2.getPosition() - F2Shoot) < servoTolerance;
                            if (f1Ready && f2Ready || shootTimer.milliseconds() > 2000) {
                                flicker1.setPosition(F1Rest);
                                flicker2.setPosition(F2Rest);
                            }
                        } else if (shootTimer.milliseconds() < 3200) {
                            spindexerTargetPos = p2;
                        } else if (shootTimer.milliseconds() < 4000) {
                        } else {
                            sShot = 2;
                            shootTimer.reset();
                        }
                        break;
                    case 2:
                        flywheel.setVelocity(getTickSpeed(targetRpm + 200));
                        if (shootTimer.milliseconds() < 600) {
                            flicker1.setPosition(F1Shoot);
                            flicker2.setPosition(F2Shoot);
                        } else if (shootTimer.milliseconds() < 1000) {
                            boolean f1Ready = Math.abs(flicker1.getPosition() - F1Shoot) < servoTolerance;
                            boolean f2Ready = Math.abs(flicker2.getPosition() - F2Shoot) < servoTolerance;
                            if (f1Ready && f2Ready || shootTimer.milliseconds() > 900) {
                                flicker1.setPosition(F1Rest);
                                flicker2.setPosition(F2Rest);
                            }
                        } else if (shootTimer.milliseconds() < 2500) {
                            spindexerTargetPos = p3;
                        } else if (shootTimer.milliseconds() < 3200) {
                        } else {
                            sShot = 3;
                            shootTimer.reset();
                        }
                        break;
                    case 3:
                        flywheel.setVelocity(getTickSpeed(targetRpm + 200));
                        if (shootTimer.milliseconds() < 600) {
                            flicker1.setPosition(F1Shoot);
                            flicker2.setPosition(F2Shoot);
                        } else if (shootTimer.milliseconds() < 1000) {
                            boolean f1Ready = Math.abs(flicker1.getPosition() - F1Shoot) < servoTolerance;
                            boolean f2Ready = Math.abs(flicker2.getPosition() - F2Shoot) < servoTolerance;
                            if (f1Ready && f2Ready || shootTimer.milliseconds() > 900) {
                                flicker1.setPosition(F1Rest);
                                flicker2.setPosition(F2Rest);
                            }
                        } else if (shootTimer.milliseconds() < 2000) {
                        } else {
                            sShot = 0;
                        }
                        break;
                }
            } else {
                switch(sShot) {
                    case 0:
                        flywheel.setVelocity(0);
                        flicker1.setPosition(F1Rest);
                        flicker2.setPosition(F2Rest);
                        break;
                    case 1:
                        flywheel.setVelocity(getTickSpeed(targetRpm));
                        if (shootTimer.milliseconds() < 1500) {
                        } else if (shootTimer.milliseconds() < 1900) {
                            flicker1.setPosition(F1Shoot);
                            flicker2.setPosition(F2Shoot);
                        } else if (shootTimer.milliseconds() < 2100) {
                            boolean f1Ready = Math.abs(flicker1.getPosition() - F1Shoot) < servoTolerance;
                            boolean f2Ready = Math.abs(flicker2.getPosition() - F2Shoot) < servoTolerance;
                            if (f1Ready && f2Ready || shootTimer.milliseconds() > 2000) {
                                flicker1.setPosition(F1Rest);
                                flicker2.setPosition(F2Rest);
                            }
                        } else if (shootTimer.milliseconds() < 3200) {
                            spindexerTargetPos = p2;
                        } else if (shootTimer.milliseconds() < 4000) {
                        } else {
                            sShot = 2;
                            shootTimer.reset();
                        }
                        break;
                    case 2:
                        flywheel.setVelocity(getTickSpeed(targetRpm + 200));
                        if (shootTimer.milliseconds() < 600) {
                            flicker1.setPosition(F1Shoot);
                            flicker2.setPosition(F2Shoot);
                        } else if (shootTimer.milliseconds() < 1000) {
                            boolean f1Ready = Math.abs(flicker1.getPosition() - F1Shoot) < servoTolerance;
                            boolean f2Ready = Math.abs(flicker2.getPosition() - F2Shoot) < servoTolerance;
                            if (f1Ready && f2Ready || shootTimer.milliseconds() > 900) {
                                flicker1.setPosition(F1Rest);
                                flicker2.setPosition(F2Rest);
                            }
                        } else if (shootTimer.milliseconds() < 2500) {
                            spindexerTargetPos = p1;
                        } else if (shootTimer.milliseconds() < 3200) {
                        } else {
                            sShot = 3;
                            shootTimer.reset();
                        }
                        break;
                    case 3:
                        flywheel.setVelocity(getTickSpeed(targetRpm + 200));
                        if (shootTimer.milliseconds() < 300) {
                            flicker1.setPosition(F1Shoot);
                            flicker2.setPosition(F2Shoot);
                        } else if (shootTimer.milliseconds() < 1000) {
                            boolean f1Ready = Math.abs(flicker1.getPosition() - F1Shoot) < servoTolerance;
                            boolean f2Ready = Math.abs(flicker2.getPosition() - F2Shoot) < servoTolerance;
                            if (f1Ready && f2Ready || shootTimer.milliseconds() > 900) {
                                flicker1.setPosition(F1Rest);
                                flicker2.setPosition(F2Rest);
                            }
                        } else if (shootTimer.milliseconds() < 2000) {
                        } else {
                            sShot = 0;
                        }
                        break;
                }
            }
            
            // Gradual spindexer movement
            if (sShot != 0) {
                double currentSpindexerPos = spindexer.getPosition();
                double error = spindexerTargetPos - currentSpindexerPos;
                if (Math.abs(error) > 0.01) {
                    double newPosition;
                    if (error > 0) {
                        newPosition = Math.min(currentSpindexerPos + spindexerIncrementSpeed, spindexerTargetPos);
                    } else {
                        newPosition = Math.max(currentSpindexerPos - spindexerIncrementSpeed, spindexerTargetPos);
                    }
                    spindexer.setPosition(newPosition);
                }
            }

            // Hood control
            double currentHoodPos = hood.getPosition();
            if (dpadUp2){
                 hood.setPosition(Math.min(1.0, currentHoodPos + 0.01));
             } else if (dpadDown2){
                 hood.setPosition(Math.max(0.0, currentHoodPos - 0.01));
            } 

            // Toggle turret auto-tracking
            if (y2 && !y2Pressed) {
                autoTrackingEnabled = !autoTrackingEnabled;
                lastTurretError = 0;
                integratedError = 0;
                filteredTurretError = 0;
                turretTimer.reset();
                y2Pressed = true;
            } else if (!y2) {
                y2Pressed = false;
            }

            // Mag sensor calibration
            boolean currentMagState = isMagPressed();
            if (currentMagState && !lastMagState) {
                int currentPos = turret.getCurrentPosition();
                int positionError = Math.abs(currentPos - magSensorPosition);
                if (positionError > 5) {
                    turret.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                    turret.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                    int offsetNeeded = magSensorPosition;
                    telemetry.addData("TURRET CALIBRATED", "Reset to %d", magSensorPosition);
                }
            }
            lastMagState = currentMagState;
            
            // Turret control
            int turretPosition = turret.getCurrentPosition();
            double turretPower = 0;
            
            if (autoTrackingEnabled && hasBlueGoal()) {
                double tx = getBlueGoalX();
                if (filteredTurretError == 0 && lastTurretError == 0) {
                    filteredTurretError = tx;
                } else {
                    filteredTurretError = FILTER_ALPHA * tx + (1 - FILTER_ALPHA) * filteredTurretError;
                }
                if (Math.abs(filteredTurretError) > TURRET_DEADBAND) {
                    double compensatedError = filteredTurretError * turretGearRatio;
                    double pTerm = compensatedError * KP_TURRET;
                    double dt = turretTimer.seconds();
                    double dTerm = 0;
                    if (dt > 0.01 && dt < 1.0) {
                        double errorChange = (filteredTurretError - lastTurretError) / dt;
                        dTerm = errorChange * KD_TURRET;
                    }
                    integratedError += filteredTurretError * dt;
                    integratedError = Math.max(-0.2, Math.min(0.2, integratedError));
                    double iTerm = integratedError * KI_TURRET;
                    turretPower = pTerm + iTerm + dTerm;
                    turretPower = Math.max(-MAX_TURRET_SPEED, Math.min(MAX_TURRET_SPEED, turretPower));
                    lastTurretError = filteredTurretError;
                    turretTimer.reset();
                } else {
                    turretPower = 0;
                    lastTurretError = filteredTurretError;
                }
            } else {
                lastTurretError = 0;
                integratedError = 0;
                filteredTurretError = 0;
            }

            // Turret safety limits
            if (limitsEnabled) {
                if (turretPosition <= turretMinLimit && turretPower < 0) {
                    turretPower = 0;
                }
                if (turretPosition >= turretMaxLimit && turretPower > 0) {
                    turretPower = 0;
                }
            }
            turret.setPower(turretPower);

            // Telemetry
            telemetry.addData("=== DRIVE ===", "");
            telemetry.addData("Drive", String.format("%.2f", drive));
            telemetry.addData("Strafe", String.format("%.2f", strafe));
            telemetry.addData("Turn", String.format("%.2f", turn));

            telemetry.addData("=== LIMELIGHT ===", "");
            telemetry.addData("Blue Goal Visible", hasBlueGoal() ? "YES" : "NO");
            if (hasBlueGoal()) {
                telemetry.addData("Target X Error", String.format("%.2f°", getBlueGoalX()));
            }
            
            telemetry.addData("=== TURRET ===", "");
            telemetry.addData("Turret Position", turret.getCurrentPosition());
            telemetry.addData("Turret Power", String.format("%.2f", turretPower));
            telemetry.addData("Auto Tracking", autoTrackingEnabled ? "ON" : "OFF");
            
            telemetry.addData("=== SHOOTER ===", "");
            telemetry.addData("Shot State", sShot == 0 ? "IDLE" : "Shot " + sShot);
            telemetry.addData("Flywheel Velocity", String.format("%.0f", flywheel.getVelocity()));
            telemetry.addData("Spindexer Pos", String.format("%.2f", spindexer.getPosition()));
            
            displayTelemetry(this);
            telemetry.update();
 
        }

    }

    private double getTickSpeed(double rpm) {
        return rpm * 28 / 60;
    }

}
