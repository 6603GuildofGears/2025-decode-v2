package org.firstinspires.ftc.teamcode.pedroPathing.TeleOp;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.hardware.limelightvision.LLResult;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;


import static org.firstinspires.ftc.teamcode.pedroPathing.Pipelines.Motor_PipeLine.*;
import static org.firstinspires.ftc.teamcode.pedroPathing.Pipelines.Servo_Pipeline.*;
import static org.firstinspires.ftc.teamcode.pedroPathing.Pipelines.Limelight_Pipeline.*;
import static org.firstinspires.ftc.teamcode.pedroPathing.Pipelines.Sensor.*;
import static org.firstinspires.ftc.teamcode.pedroPathing.TeleOp.TurretConfig.*;
import org.firstinspires.ftc.teamcode.pedroPathing.Pipelines.SpindexerController;



@TeleOp(name="Cassius Blue", group="TeleOp")
public class Cassius_Blue extends LinearOpMode {



    @Override 
    public void runOpMode() throws InterruptedException {

        // pipelines 
        intMotors(this);
        intServos(this);
        initLimelight(this);
        initSensors(this);

        // Odometry — Pinpoint localizer for turret field-relative aiming
        Follower follower = Constants.createFollower(hardwareMap);
        // Set start pose (adjust to your actual start position on the field)
        follower.setStartingPose(new Pose(72, 72, 0)); // center, facing +X
        
        // Reset turret encoder to 0 at current position (should be centered manually before init)
        turret.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        turret.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        // IMU for yaw-rate feedforward (counter-rotates turret when chassis spins)
        IMU imu = hardwareMap.get(IMU.class, "imu");
        imu.initialize(new IMU.Parameters(new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.UP,
                RevHubOrientationOnRobot.UsbFacingDirection.FORWARD)));
        imu.resetYaw();

        telemetry.addData("Status", "Hardware initialized");
        telemetry.update();


        // put all initlization code here

        double gear = 1.25; // speed modifier for drive train
        
        // Turret tracking toggle
        boolean autoTrackingEnabled = true; // Default to auto
        boolean y2Pressed = false; // Debounce for Y button

        // Spindexer controller — handles intake detection, slot tracking, and shooting
        SpindexerController sdx = new SpindexerController();
        sdx.setFlickerPositions(0.1, 0.0875, 0.5, 0.5);
        flicker1.setPosition(0.1);
        flicker2.setPosition(0.0875);


       



    double baseRpm = 3000; // default RPM for shooter (NOTE: 'intake' variable is actually the shooter motor)
    double targetRpm = baseRpm; // updated from Limelight lookup when available
    double targetHoodPos = hood.getPosition();
    // Limelight distance config (inches/degrees)
    double cameraHeight = 11.125; // Camera lens center height in inches
    double cameraMountAngle = 22.85; // Camera angle from horizontal (calibrated)
    double targetHeight = 29.5; // AprilTag center height in inches
      
        // Turret safety limits (FOUND FROM TESTING!)
        int turretMinLimit = 0; // Left limit
        int turretMaxLimit = 850;  // Right limit
        boolean limitsEnabled = true; // Limits are now active
        
        // Mag sensor calibration position
        int magSensorPosition = -149; // Turret position when mag sensor triggers (-145 to -153 range)
        boolean lastMagState = false; // Track mag sensor state changes

        // Turret PID values are now in TurretConfig.java for live tuning via Pedro Pathing Panels
        // kRot: IMU yaw-rate feedforward gain (power per °/s of chassis rotation)
        double kRot = 0.00378;
        
        // Turret tracking state variables (matches PID tuner)
        double filteredTurretError = 0;
        double integratedError = 0;
        double previousError = 0;
        double filteredDeriv = 0;       // Low-pass filtered derivative
        boolean filterInited = false;   // First-reading flag
        double D_FILTER_ALPHA = 0.5;    // Derivative filter (less lag)
        double MIN_TURRET_POWER = 0.04; // Static friction compensation
        int LIMIT_SLOW_ZONE = 30;       // Ramp down near limits
        ElapsedTime turretTimer = new ElapsedTime();
        ElapsedTime targetLostTimer = new ElapsedTime();
        ElapsedTime acquireTimer = new ElapsedTime(); // Tracks time since target first reacquired
        boolean hadTargetLastFrame = false;            // Detect target-found transition
        double ACQUIRE_SLOW_SEC = 0.12;                 // Seconds to run at reduced speed
        double ACQUIRE_SPEED_LIMIT = 0.20;             // Max turret power during acquire window

        // Panels telemetry
        TelemetryManager telemetryM = PanelsTelemetry.INSTANCE.getTelemetry();



        waitForStart();
        spindexer.setPosition(SpindexerController.P1); // start at P1 once
        while (opModeIsActive()) {
               // Update odometry pose every loop (even when Limelight is tracking)
               follower.update();

               boolean LStickIn2 = gamepad2.left_stick_button;
                boolean RStickIn2 = gamepad2.right_stick_button;
                boolean LBumper1 = gamepad1.left_bumper;
                boolean RBumper1 = gamepad1.right_bumper;
                double LStickY = -gamepad1.left_stick_y;
                double LStickX = gamepad1.left_stick_x;
                double RStickY = gamepad1.right_stick_y;
                double RStickX = gamepad1.right_stick_x;

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
                    SetPower(gear, -gear, gear, -gear);

                } else if (LTrigger1 > 0.25) {
                    SetPower(gear, -gear, gear, -gear);

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


            
            telemetry.addData("spindexer pos", spindexer.getPosition());
            if(dpadRight2){
                double CPoS = spindexer.getPosition();
                spindexer.setPosition(CPoS + 0.005);;
            } else if (dpadLeft2){
                double CPoS = spindexer.getPosition();
                spindexer.setPosition(CPoS - 0.005);;
            }


            // Intake with automatic spindexer rotation via SpindexerController
            boolean intakeRequested = RTrigger1 > 0.1;
            boolean runIntakeMotor = sdx.updateIntake(intakeRequested);

            if (runIntakeMotor) {
                intake.setPower(0.75); // intake in (spindexer auto-rotates on ball detect)
            } else if (RBumper1) {
                intake.setPower(-1); // intake out (reverse)
            } else {
                intake.setPower(0);
            }


            // Limelight-based shooter tuning (distance -> rpm + hood)
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
            if (hasDistance) {
                // Lookup table is primary — always applies when target is visible
                ShooterLookup.Result tuned = ShooterLookup.lookup(distanceInches);
                targetRpm = tuned.rpm;
                targetHoodPos = tuned.hoodPos;
                hood.setPosition(targetHoodPos);
                sdx.setShootRpm(targetRpm);
            } else {
                // Manual hood control is secondary — only when no target
                targetRpm = baseRpm;
                sdx.setShootRpm(baseRpm);
                double currentHoodPos = hood.getPosition();
                if (dpadUp2){
                     hood.setPosition(Math.min(1.0, currentHoodPos + 0.01)); // Slide up
                 } else if (dpadDown2){
                     hood.setPosition(Math.max(0.0, currentHoodPos - 0.01)); // Slide down
                }
            }

            // Shooting sequence — SpindexerController handles everything:
            //   slot tracking, skip empties, smart direction, flicker timing
            //   RBumper2 = shoot, LBumper2 = kill switch
            sdx.updateShoot(RBumper2, LBumper2, flywheel); 

            // Toggle turret auto-tracking with Y button (gamepad2)
            if (y2 && !y2Pressed) {
                autoTrackingEnabled = !autoTrackingEnabled;
                // Reset tracking state when toggling
                integratedError = 0;
                previousError = 0;
                filteredDeriv = 0;
                filteredTurretError = 0;
                filterInited = false;
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
            
            // Turret control — PID + IMU yaw-rate feedforward (from PID tuner)
            int turretPosition = turret.getCurrentPosition();
            double turretPower = 0;

            // IMU yaw-rate feedforward: counter-rotate turret to cancel chassis spin
            double yawRate = imu.getRobotAngularVelocity(AngleUnit.DEGREES).zRotationRate;
            double rotationFF = yawRate * kRot;

            double dt = turretTimer.seconds();
            turretTimer.reset();
            if (dt > 0.5) dt = 0.02; // guard against first-frame spike

            if (autoTrackingEnabled && hasBlueGoal()) {
                double tx = getBlueGoalX();

                // Low-pass filter — initialize on first reading to avoid lag
                if (!filterInited) {
                    filteredTurretError = tx;
                    previousError = tx;
                    filteredDeriv = 0;
                    filterInited = true;
                } else {
                    filteredTurretError = FILTER_ALPHA * tx + (1.0 - FILTER_ALPHA) * filteredTurretError;
                }

                if (Math.abs(filteredTurretError) > TURRET_DEADBAND) {
                    double error = filteredTurretError;

                    // Integral with anti-windup
                    integratedError += error * dt;
                    double maxIntegral = MAX_TURRET_SPEED / Math.max(Math.abs(KI_TURRET), 0.0001);
                    integratedError = Math.max(-maxIntegral, Math.min(maxIntegral, integratedError));

                    // Filtered derivative — low-pass to reject Limelight noise
                    double rawDeriv = (error - previousError) / dt;
                    filteredDeriv = D_FILTER_ALPHA * rawDeriv + (1.0 - D_FILTER_ALPHA) * filteredDeriv;
                    previousError = error;

                    turretPower = (KP_TURRET * error) + (KI_TURRET * integratedError) + (KD_TURRET * filteredDeriv);

                    // Static friction compensation: ensure minimum power to actually move
                    if (Math.abs(turretPower) > 0.001 && Math.abs(turretPower) < MIN_TURRET_POWER) {
                        turretPower = Math.signum(turretPower) * MIN_TURRET_POWER;
                    }
                } else {
                    // Within deadband — centered, decay integral to prevent stale bias
                    turretPower = 0;
                    integratedError *= 0.8;
                    previousError = filteredTurretError;
                }

                // Always apply yaw-rate feedforward (cancels chassis rotation even when centered)
                turretPower += rotationFF;

                // Acquisition slowdown: if we just found the target, limit speed briefly
                boolean acquiring = (!hadTargetLastFrame);
                if (acquiring) {
                    acquireTimer.reset();
                }
                double currentSpeedLimit = MAX_TURRET_SPEED;
                if (acquireTimer.seconds() < ACQUIRE_SLOW_SEC) {
                    currentSpeedLimit = ACQUIRE_SPEED_LIMIT;
                }
                turretPower = Math.max(-currentSpeedLimit, Math.min(currentSpeedLimit, turretPower));

                hadTargetLastFrame = true;
                targetLostTimer.reset();

            } else if (autoTrackingEnabled && !hasBlueGoal()) {
                // Reset PID state
                integratedError = 0;
                previousError = 0;
                filteredDeriv = 0;
                filterInited = false;
                hadTargetLastFrame = false;

                // Start with yaw-rate compensation
                turretPower = rotationFF;

                double timeLost = targetLostTimer.seconds();
                if (timeLost > 0.15) {
                    // Odometry-guided aim after brief grace period
                    Pose pose = follower.getPose();
                    double dx = BLUE_GOAL_X - pose.getX();
                    double dy = BLUE_GOAL_Y - pose.getY();
                    double fieldAngleRad = Math.atan2(dy, dx);

                    double robotRelativeRad = fieldAngleRad - pose.getHeading();
                    robotRelativeRad = Math.atan2(Math.sin(robotRelativeRad), Math.cos(robotRelativeRad));
                    double robotRelativeDeg = Math.toDegrees(robotRelativeRad);

                    double targetTicks = TURRET_FORWARD_TICKS + (robotRelativeDeg * TICKS_PER_TURRET_DEG);
                    targetTicks = Math.max(turretMinLimit, Math.min(turretMaxLimit, targetTicks));

                    double tickError = targetTicks - turretPosition;

                    if (Math.abs(tickError) > 3) {
                        turretPower = Math.signum(tickError) * Math.min(ODO_AIM_POWER,
                                Math.abs(tickError) * 0.005);
                        turretPower += rotationFF;
                    }

                    turretPower = Math.max(-MAX_TURRET_SPEED, Math.min(MAX_TURRET_SPEED, turretPower));

                    telemetry.addData("Odo Aim", String.format("%.1f° → tick %.0f (err %.0f)",
                            robotRelativeDeg, targetTicks, tickError));
                }
            } else {
                // Tracking disabled
                integratedError = 0;
                previousError = 0;
                filteredDeriv = 0;
                filterInited = false;
                filteredTurretError = 0;
                hadTargetLastFrame = false;
            }

            // Safety limits with slow zone (ramps down near limits instead of hard stop)
            if (limitsEnabled) {
                if (turretPosition >= turretMaxLimit && turretPower > 0) {
                    turretPower = 0;
                } else if (turretPower > 0 && turretPosition > turretMaxLimit - LIMIT_SLOW_ZONE) {
                    double s = (double)(turretMaxLimit - turretPosition) / LIMIT_SLOW_ZONE;
                    turretPower *= Math.max(0.0, Math.min(1.0, s));
                }
                if (turretPosition <= turretMinLimit && turretPower < 0) {
                    turretPower = 0;
                } else if (turretPower < 0 && turretPosition < turretMinLimit + LIMIT_SLOW_ZONE) {
                    double s = (double)(turretPosition - turretMinLimit) / LIMIT_SLOW_ZONE;
                    turretPower *= Math.max(0.0, Math.min(1.0, s));
                }
            }

            turret.setPower(turretPower);

            // Display telemetry
            telemetry.addData("=== LIMELIGHT STATUS ===", "");
            telemetry.addData("LL Connected", hasTarget() ? "YES" : "CHECKING...");
            telemetry.addData("Blue Goal Visible", hasBlueGoal() ? "YES" : "NO");
            if (hasBlueGoal()) {
                telemetry.addData("Target X Error", String.format("%.2f°", getBlueGoalX()));
            }
            
            telemetry.addData("=== TURRET ===", "");
            telemetry.addData("Turret Position", turret.getCurrentPosition());
            telemetry.addData("Turret Power", String.format("%.2f", turretPower));
            telemetry.addData("Auto Tracking", autoTrackingEnabled ? "ON" : "OFF");
            telemetry.addData("Filtered Error", String.format("%.2f°", filteredTurretError));
            telemetry.addData("Integrated Error", String.format("%.3f", integratedError));
            telemetry.addData("Yaw Rate", String.format("%.1f °/s", yawRate));
            telemetry.addData("--- PID TUNING (Panels) ---", "");
            telemetry.addData("KP", String.format("%.3f", KP_TURRET));
            telemetry.addData("KI", String.format("%.3f", KI_TURRET));
            telemetry.addData("KD", String.format("%.3f", KD_TURRET));
            telemetry.addData("kRot", String.format("%.4f", kRot));
            
            telemetry.addData("=== SHOOTER ===", "");
            telemetry.addData("Flywheel Velocity", String.format("%.0f", flywheel.getVelocity()));
            sdx.addTelemetry(telemetry);
            
            // === LOOKUP TABLE TEST — always show what the table would output ===
            telemetry.addData("=== LOOKUP TABLE TEST ===", "");
            telemetry.addData("LL Distance (in)", hasDistance ? String.format("%.1f", distanceInches) : "NO TARGET");
            if (hasDistance) {
                ShooterLookup.Result preview = ShooterLookup.lookup(distanceInches);
                telemetry.addData("Lookup RPM", String.format("%.0f", preview.rpm));
                telemetry.addData("Lookup Hood Pos", String.format("%.3f", preview.hoodPos));
                telemetry.addData("Active RPM", String.format("%.0f", targetRpm));
                telemetry.addData("Active Hood", String.format("%.3f", hood.getPosition()));
                telemetry.addData("RPM Match?", Math.abs(targetRpm - preview.rpm) < 1 ? "YES ✓" : "NO (manual/base)");
            }
            
            telemetry.addData("=== LIMELIGHT DEBUG ===", "");
            LLResult llResult = getLatestResult();
            if (llResult != null && llResult.isValid() && llResult.getFiducialResults() != null && !llResult.getFiducialResults().isEmpty()) {
                int tagId = (int) llResult.getFiducialResults().get(0).getFiducialId();
                double tx = llResult.getFiducialResults().get(0).getTargetXDegrees();
                double ty = llResult.getFiducialResults().get(0).getTargetYDegrees();
                telemetry.addData("Tag ID", tagId);
                telemetry.addData("TX", String.format("%.2f°", tx));
                telemetry.addData("TY", String.format("%.2f°", ty));
                telemetry.addData("Targets", llResult.getFiducialResults().size());
            } else {
                telemetry.addData("Tag ID", "NONE");
                telemetry.addData("TX", "--");
                telemetry.addData("TY", "--");
                telemetry.addData("Targets", 0);
            }
            
            // Push key data to Panels
            telemetryM.debug("Blue Goal: " + (hasBlueGoal() ? "YES" : "NO"));
            telemetryM.debug("Turret Pos: " + turret.getCurrentPosition() + " | Pwr: " + String.format("%.2f", turretPower));
            telemetryM.debug("Filtered Err: " + String.format("%.2f°", filteredTurretError) + " | Yaw: " + String.format("%.1f°/s", yawRate));
            telemetryM.debug("PID: P=" + String.format("%.4f", KP_TURRET) + " I=" + String.format("%.4f", KI_TURRET) + " D=" + String.format("%.4f", KD_TURRET) + " R=" + String.format("%.4f", kRot));
            telemetryM.debug("Distance: " + (hasDistance ? String.format("%.1f in", distanceInches) : "--"));
            telemetryM.debug("RPM: " + String.format("%.0f", targetRpm) + " | Hood: " + String.format("%.3f", hood.getPosition()));
            telemetryM.debug("Flywheel: " + String.format("%.0f", flywheel.getVelocity()));
            telemetryM.update(telemetry);
            
            displayTelemetry(this); // Shows Limelight FPS and additional info
 
        }

    }

    

    // Helper method to convert RPM to ticks per second
    private double getTickSpeed(double rpm) {
        return rpm * 28 / 60; // 28 ticks per revolution, 60 seconds per minute
    }

}

