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
      
        // Turret safety limits in DEGREES (converted to ticks internally)
        double turretMinDeg = 0;      // Left limit (degrees)
        double turretMaxDeg = 322;    // Right limit (degrees) — was 850 ticks
        boolean limitsEnabled = true;
        double LIMIT_SLOW_ZONE_DEG = 11.4; // Ramp-down zone in degrees (~30 ticks)
        
        // Mag sensor = turret home (position 0). Turret starts here at init.
        boolean lastMagState = false; // Track mag sensor state changes

        // Turret values in TurretConfig.java (KP_TURRET, K_HEADING_LOCK, etc.)
        
        // Heading lock state
        double lockedHeading = 0;
        boolean headingLocked = false;

        // Position hold state — locks turret in place when idle
        int holdPosition = 0;
        boolean positionHeld = false;
        double K_HOLD = 0.005; // P gain for position hold (ticks)

        // Panels telemetry
        TelemetryManager telemetryM = PanelsTelemetry.INSTANCE.getTelemetry();

        // Init loop — show spindexer/sensor telemetry while waiting for start
        while (!isStarted() && !isStopRequested()) {
            updateSensors();
            telemetry.addData("=== INIT ===", "Waiting for Start");
            sdx.addTelemetry(telemetry);
            telemetry.update();
        }

        if (isStopRequested()) return;
        spindexer.setPosition(SpindexerController.P1); // start at P1 once
        while (opModeIsActive()) {
               // Poll Limelight ONCE per loop — all getters below see the SAME frame
               pollOnce();
               boolean LStickIn2 = gamepad2.left_stick_button;
                boolean RStickIn2 = gamepad2.right_stick_button;
                boolean LBumper1 = gamepad1.left_bumper;
                boolean RBumper1 = gamepad1.right_bumper;
                double LStickY = gamepad1.left_stick_y;
                double LStickX = -gamepad1.left_stick_x;
                double RStickY = gamepad1.right_stick_y;
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
                spindexer.setPosition(CPoS + 0.01);;
            } else if (dpadLeft2){
                double CPoS = spindexer.getPosition();
                spindexer.setPosition(CPoS - 0.01);;
            }


            // Intake with automatic spindexer rotation via SpindexerController
            boolean intakeRequested = RTrigger1 > 0.1;
            boolean runIntakeMotor = sdx.updateIntake(intakeRequested);

            if (runIntakeMotor) {
                intake.setPower(0.75); // intake in (spindexer auto-rotates on ball detect)
            } else if (sdx.isShooting()) {
                intake.setPower(0.25); // slow feed during shoot sequence
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

            // Mag sensor calibration — DISABLED at runtime to prevent position jumps
            // (encoder resets cause brief power loss and tracking hiccups)
            // Calibration happens at init via STOP_AND_RESET_ENCODER above.
            boolean currentMagState = isMagPressed();
            // if (currentMagState && !lastMagState) {
            //     turret.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            //     turret.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            // }
            lastMagState = currentMagState;
            
            // Turret control — simple P-control + heading lock + manual override
            int turretPosition = turret.getCurrentPosition();
            double turretPower = 0;

            // Cache Limelight data ONCE per loop
            boolean blueGoalVisible = hasBlueGoal();
            double blueGoalTx = blueGoalVisible ? getBlueGoalX() : 0;

            // Read IMU heading for heading lock
            double currentHeading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);
            double headingError = 0;

            // Manual turret control — gamepad2 left stick X
            boolean manualTurret = Math.abs(LStickX2) > 0.1;

            if (manualTurret) {
                // Manual override — direct stick control
                turretPower = LStickX2 * MAX_TURRET_SPEED;
                headingLocked = false;
                positionHeld = false;

            } else if (blueGoalVisible) {
                double tx = blueGoalTx;
                positionHeld = false;

                if (Math.abs(tx) > TURRET_DEADBAND) {
                    // P-control: power proportional to error
                    turretPower = KP_TURRET * tx;

                    // Min power to overcome static friction
                    if (Math.abs(turretPower) < 0.02) {
                        turretPower = Math.signum(turretPower) * 0.02;
                    }
                } else {
                    // On target — lock heading
                    lockedHeading = currentHeading;
                    headingLocked = true;
                    turretPower = 0;
                }

                // Heading lock: counter-rotate when chassis spins
                if (headingLocked) {
                    headingError = currentHeading - lockedHeading;
                    while (headingError > 180) headingError -= 360;
                    while (headingError < -180) headingError += 360;
                    turretPower += headingError * K_HEADING_LOCK;
                }

                turretPower = Math.max(-MAX_TURRET_SPEED, Math.min(MAX_TURRET_SPEED, turretPower));

            } else if (headingLocked) {
                // No target — heading lock holds aim
                headingError = currentHeading - lockedHeading;
                while (headingError > 180) headingError -= 360;
                while (headingError < -180) headingError += 360;
                turretPower = headingError * K_HEADING_LOCK;
                turretPower = Math.max(-MAX_TURRET_SPEED, Math.min(MAX_TURRET_SPEED, turretPower));
                positionHeld = false;

            } else {
                // No target, no heading lock, no manual — HOLD POSITION
                if (!positionHeld) {
                    holdPosition = turretPosition;
                    positionHeld = true;
                }
                int posError = turretPosition - holdPosition;
                turretPower = -posError * K_HOLD;
                turretPower = Math.max(-0.15, Math.min(0.15, turretPower));
            }

            // HARD limit enforcement — turret CANNOT move past limits under any circumstance
            if (limitsEnabled) {
                double turretDeg = turretPosition / TICKS_PER_DEG;

                // ---- HARD STOPS: kill all positive/negative power at or past limits ----
                if (turretDeg >= turretMaxDeg && turretPower > 0) {
                    turretPower = 0;
                }
                if (turretDeg <= turretMinDeg && turretPower < 0) {
                    turretPower = 0;
                }

                // ---- SLOW ZONES: ramp down power approaching limits ----
                if (turretPower > 0 && turretDeg > turretMaxDeg - LIMIT_SLOW_ZONE_DEG) {
                    double s = (turretMaxDeg - turretDeg) / LIMIT_SLOW_ZONE_DEG;
                    turretPower *= Math.max(0.0, Math.min(1.0, s));
                }
                if (turretPower < 0 && turretDeg < turretMinDeg + LIMIT_SLOW_ZONE_DEG) {
                    double s = (turretDeg - turretMinDeg) / LIMIT_SLOW_ZONE_DEG;
                    turretPower *= Math.max(0.0, Math.min(1.0, s));
                }

                // ---- OVERSHOOT RECOVERY: if past limit, actively push back ----
                if (turretDeg > turretMaxDeg) {
                    turretPower = -0.15;  // push back toward safe zone
                }
                if (turretDeg < turretMinDeg) {
                    turretPower = 0.15;   // push back toward safe zone
                }
            }

            turret.setPower(turretPower);

            // Display telemetry (use cached values — same data the PID used)
            telemetry.addData("=== LIMELIGHT STATUS ===", "");
            telemetry.addData("LL Connected", hasTarget() ? "YES" : "CHECKING...");
            telemetry.addData("Blue Goal Visible", blueGoalVisible ? "YES" : "NO");
            if (blueGoalVisible) {
                telemetry.addData("Target X Error", String.format("%.2f°", blueGoalTx));
            }
            
            telemetry.addData("=== TURRET ===", "");
            telemetry.addData("Turret Angle", String.format("%.1f°", turretPosition / TICKS_PER_DEG));
            telemetry.addData("Turret Power", String.format("%.2f", turretPower));
            telemetry.addData("Mode", manualTurret ? "MANUAL" :
                    (blueGoalVisible ? "TRACKING" :
                    (headingLocked ? "HEADING LOCK" : "IDLE")));
            telemetry.addData("Heading Lock", headingLocked ?
                    String.format("%.1f° (err=%.1f°)", lockedHeading, headingError) : "OFF");
            telemetry.addData("--- PID TUNING (Panels) ---", "");
            telemetry.addData("KP", String.format("%.4f", KP_TURRET));
            telemetry.addData("K_HL", String.format("%.4f", K_HEADING_LOCK));
            
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
            telemetryM.debug("Turret: " + String.format("%.1f°", turretPosition / TICKS_PER_DEG) + " | Pwr: " + String.format("%.2f", turretPower) + " | " + (manualTurret ? "MANUAL" : (blueGoalVisible ? "TRACK" : (headingLocked ? "HL" : "IDLE"))));
            telemetryM.debug("KP=" + String.format("%.4f", KP_TURRET) + " K_HL=" + String.format("%.4f", K_HEADING_LOCK));
            telemetryM.debug("Distance: " + (hasDistance ? String.format("%.1f in", distanceInches) : "--"));
            telemetryM.debug("RPM: " + String.format("%.0f", targetRpm) + " | Hood: " + String.format("%.3f", hood.getPosition()));
            telemetryM.debug("Flywheel: " + String.format("%.0f", flywheel.getVelocity()));
            displayTelemetry(this); // Shows Limelight FPS and additional info
            telemetry.update(); // Push telemetry to Driver Station
            telemetryM.update(); // Push Panels data separately
 
        }

    }

    

    // Helper method to convert RPM to ticks per second
    private double getTickSpeed(double rpm) {
        return rpm * 28 / 60; // 28 ticks per revolution, 60 seconds per minute
    }

}

