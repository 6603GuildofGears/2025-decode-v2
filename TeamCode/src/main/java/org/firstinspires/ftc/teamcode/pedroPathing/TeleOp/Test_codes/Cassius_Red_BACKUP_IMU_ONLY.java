package org.firstinspires.ftc.teamcode.pedroPathing.TeleOp.Test_codes;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.hardware.limelightvision.LLResult;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AngularVelocity;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;

import static org.firstinspires.ftc.teamcode.pedroPathing.Pipelines.Motor_PipeLine.*;
import static org.firstinspires.ftc.teamcode.pedroPathing.Pipelines.Servo_Pipeline.*;
import static org.firstinspires.ftc.teamcode.pedroPathing.Pipelines.Limelight_Pipeline.*;
import static org.firstinspires.ftc.teamcode.pedroPathing.Pipelines.Sensor.*;
import org.firstinspires.ftc.teamcode.pedroPathing.TeleOp.ShooterLookup;
import static org.firstinspires.ftc.teamcode.pedroPathing.TeleOp.TurretConfig.*;
import org.firstinspires.ftc.teamcode.pedroPathing.Pipelines.SpindexerController;



@Disabled
@TeleOp(name="Cassius Red BACKUP", group="TeleOp")
public class Cassius_Red_BACKUP_IMU_ONLY extends LinearOpMode {



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
        // Hub is mounted with logo facing BACKWARD, USB ports facing UP (~75° tilt)
        IMU imu = hardwareMap.get(IMU.class, "imu");
        imu.initialize(new IMU.Parameters(new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.BACKWARD,
                RevHubOrientationOnRobot.UsbFacingDirection.UP)));
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


       



    double baseRpm = 3000; // default RPM for shooter
    double targetRpm = baseRpm; // updated from Limelight lookup when available
    double targetHoodPos = hood.getPosition();
    // Limelight distance config (inches/degrees)
    double cameraHeight = 11.125; // Camera lens center height in inches
    double cameraMountAngle = 22.85; // Camera angle from horizontal (calibrated)
    double targetHeight = 29.5; // AprilTag center height in inches
      
        // Turret safety limits in DEGREES (converted to ticks internally)
        double turretMinDeg = 5;       // Left limit (degrees) — avoid cable strain
        double turretMaxDeg = 300;     // Right limit (degrees)
        boolean limitsEnabled = true;
        double LIMIT_SLOW_ZONE_DEG = 15; // Ramp-down zone in degrees
        
        // Mag sensor = turret home (position 0). Turret starts here at init.
        boolean lastMagState = false; // Track mag sensor state changes

        // Turret values in TurretConfig.java (KP_TURRET, KI_TURRET, KD_TURRET, etc.)

        // === 3-LAYER TURRET TRACKING STATE ===
        // Layer 1: IMU yaw-rate feedforward (proactive — cancels chassis rotation instantly)
        // Layer 2: Vision PID (reactive — cleans up residual error from Limelight tx)
        // Layer 3: Field-angle lock (fallback — holds aim direction when target is blocked)

        // PID state (Layer 2)
        double pidIntegral = 0;
        double pidLastError = 0;
        double filteredTx = 0;
        boolean pidInitialized = false;
        ElapsedTime pidTimer = new ElapsedTime();

        // Field-angle lock state (Layer 3)
        double lockedFieldAngle = 0;   // Field-space angle the turret was pointing when target was last seen
        boolean hasFieldLock = false;  // True once we've captured a lock angle

        // Position hold for manual mode
        int holdPosition = 0;
        boolean positionHeld = false;
        double K_HOLD = 0.005; // P gain for position hold (ticks)

        // Panels telemetry
        TelemetryManager telemetryM = PanelsTelemetry.INSTANCE.getTelemetry();

        // ── Auto-home: rotate turret left until mag sensor triggers, then zero encoder ──
        boolean homed = false;
        telemetry.addData("=== INIT ===", "Homing turret...");
        telemetry.update();
        // If already on mag, skip movement
        updateSensors();
        if (isMagPressed()) {
            turret.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            turret.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            homed = true;
        }
        while (!homed && !isStarted() && !isStopRequested()) {
            updateSensors();
            if (isMagPressed()) {
                turret.setPower(0);
                turret.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                turret.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                homed = true;
            } else {
                turret.setPower(-0.15); // slow left toward mag sensor
            }
            telemetry.addData("=== INIT ===", "Homing turret...");
            telemetry.addData("Mag", isMagPressed());
            telemetry.update();
        }
        turret.setPower(0); // safety stop

        // Init loop — show spindexer/sensor telemetry while waiting for start
        while (!isStarted() && !isStopRequested()) {
            updateSensors();
            telemetry.addData("=== INIT ===", homed ? "HOMED ✓ — Ready" : "HOME FAILED");
            telemetry.addData("Turret Pos", turret.getCurrentPosition());
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

            // Drive code 

      
               if (Math.abs(LStickX) > 0 || Math.abs(LStickY) > 0 || Math.abs(RStickX) > 0) {
                    double rotation = 0;

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
                spindexer.setPosition(CPoS + 0.03);
            } else if (dpadLeft2){
                double CPoS = spindexer.getPosition();
                spindexer.setPosition(CPoS - 0.03);
            }


            // Intake with automatic spindexer rotation via SpindexerController
            boolean intakeRequested = RTrigger1 > 0.1;
            boolean runIntakeMotor = sdx.updateIntake(intakeRequested);

            if (runIntakeMotor) {
                intake.setPower(1); // intake in (spindexer auto-rotates on ball detect)
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
                     hood.setPosition(Math.min(1.0, currentHoodPos + 0.01));
                 } else if (dpadDown2){
                     hood.setPosition(Math.max(0.0, currentHoodPos - 0.01));
                }
            }

            // Shooting sequence — SpindexerController handles everything
            sdx.updateShoot(RBumper2, LBumper2, flywheel); 

            // Mag sensor state tracking
            boolean currentMagState = isMagPressed();
            lastMagState = currentMagState;

            // ================================================================
            //  3-LAYER TURRET TRACKING
            //  Layer 1: IMU yaw-rate feedforward (PROACTIVE — instant)
            //  Layer 2: Vision PID on Limelight tx (REACTIVE — cleans up residual)
            //  Layer 3: Field-angle lock (FALLBACK — holds aim when blocked)
            // ================================================================

            int turretPosition = turret.getCurrentPosition();
            double turretDeg = turretPosition / TICKS_PER_DEG;
            double turretPower = 0;
            String turretMode = "IDLE";

            // --- Read IMU data (used by Layer 1 and Layer 3) ---
            double yawRate = imu.getRobotAngularVelocity(AngleUnit.DEGREES).zRotationRate;
            double robotHeading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);

            // --- Layer 1: Yaw-rate feedforward (ALWAYS active, even when no target) ---
            // Sign: positive yawRate (CCW robot spin) needs positive turret power
            double yawFF = yawRate * K_YAW_FF;

            // --- Cache Limelight data ONCE per loop — TARGET RED GOAL ---
            boolean redGoalVisible = hasRedGoal();
            double redGoalTx = redGoalVisible ? getRedGoalX() : 0;

            // --- Manual turret control — gamepad2 left stick X ---
            double manualInput = LStickX2 / 1.75;
            boolean manualTurret = Math.abs(manualInput) > 0.03;

            // --- PID delta time ---
            double dt = pidTimer.seconds();
            pidTimer.reset();
            if (dt <= 0 || dt > 0.5) dt = 0.02;

            if (manualTurret) {
                // === MANUAL OVERRIDE ===
                turretPower = manualInput * MAX_TURRET_SPEED;
                positionHeld = false;
                hasFieldLock = false;
                pidIntegral = 0;
                pidLastError = 0;
                pidInitialized = false;
                turretMode = "MANUAL";

            } else if (redGoalVisible) {
                // === TARGET VISIBLE: Layer 1 (FF) + Layer 2 (Vision PID) ===
                double tx = redGoalTx;
                positionHeld = false;
                turretMode = "TRACKING";

                // -- Layer 2: Vision PID --
                if (!pidInitialized) {
                    filteredTx = tx;
                    pidLastError = tx;
                    pidInitialized = true;
                } else {
                    filteredTx = FILTER_ALPHA * tx + (1.0 - FILTER_ALPHA) * filteredTx;
                }

                double error = filteredTx;

                if (Math.abs(error) > TURRET_DEADBAND) {
                    double pTerm = KP_TURRET * error;

                    pidIntegral += error * dt;
                    double maxIntegral = MAX_TURRET_SPEED / Math.max(Math.abs(KI_TURRET), 0.0001);
                    pidIntegral = Math.max(-maxIntegral, Math.min(maxIntegral, pidIntegral));
                    double iTerm = KI_TURRET * pidIntegral;

                    double dTerm = KD_TURRET * ((error - pidLastError) / dt);

                    double visionPID = pTerm + iTerm + dTerm;

                    if (Math.abs(visionPID) > 0.001 && Math.abs(visionPID) < 0.02) {
                        visionPID = Math.signum(visionPID) * 0.02;
                    }

                    turretPower = visionPID + yawFF;
                } else {
                    turretPower = yawFF;
                    pidIntegral *= 0.9;
                    turretMode = "ON TARGET";
                }

                pidLastError = error;

                // --- Update field-angle lock ---
                lockedFieldAngle = normalizeAngle(robotHeading + turretDeg);
                hasFieldLock = true;

                turretPower = Math.max(-MAX_TURRET_SPEED, Math.min(MAX_TURRET_SPEED, turretPower));

            } else if (hasFieldLock) {
                // === TARGET BLOCKED: Layer 1 (FF) + Layer 3 (Field Lock) ===
                turretMode = "FIELD LOCK";
                positionHeld = false;
                pidIntegral = 0;
                pidLastError = 0;
                pidInitialized = false;

                double desiredTurretDeg = normalizeAngle(lockedFieldAngle - robotHeading);
                desiredTurretDeg = Math.max(turretMinDeg, Math.min(turretMaxDeg, desiredTurretDeg));

                double lockError = normalizeAngle(desiredTurretDeg - turretDeg);
                double lockPower = lockError * K_FIELD_LOCK;

                turretPower = lockPower + yawFF;
                turretPower = Math.max(-LOCK_MAX_POWER, Math.min(LOCK_MAX_POWER, turretPower));

            } else {
                // === NO TARGET, NO LOCK — hold encoder position ===
                turretMode = "HOLD";
                if (!positionHeld) {
                    holdPosition = turretPosition;
                    positionHeld = true;
                }
                int posError = turretPosition - holdPosition;
                turretPower = -posError * K_HOLD + yawFF;
                turretPower = Math.max(-0.15, Math.min(0.15, turretPower));
            }

            // ================================================================
            //  HARD LIMIT ENFORCEMENT
            // ================================================================
            if (limitsEnabled) {
                if (turretDeg >= turretMaxDeg && turretPower > 0) {
                    turretPower = 0;
                }
                if (turretDeg <= turretMinDeg && turretPower < 0) {
                    turretPower = 0;
                }

                if (turretPower > 0 && turretDeg > turretMaxDeg - LIMIT_SLOW_ZONE_DEG) {
                    double s = (turretMaxDeg - turretDeg) / LIMIT_SLOW_ZONE_DEG;
                    turretPower *= Math.max(0.0, Math.min(1.0, s));
                }
                if (turretPower < 0 && turretDeg < turretMinDeg + LIMIT_SLOW_ZONE_DEG) {
                    double s = (turretDeg - turretMinDeg) / LIMIT_SLOW_ZONE_DEG;
                    turretPower *= Math.max(0.0, Math.min(1.0, s));
                }

                if (turretDeg > turretMaxDeg) {
                    turretPower = -0.15;
                }
                if (turretDeg < turretMinDeg) {
                    turretPower = 0.15;
                }
            }

            turret.setPower(turretPower);

            // Display telemetry
            telemetry.addData("=== LIMELIGHT STATUS ===", "");
            telemetry.addData("LL Connected", hasTarget() ? "YES" : "CHECKING...");
            telemetry.addData("Red Goal Visible", redGoalVisible ? "YES" : "NO");
            if (redGoalVisible) {
                telemetry.addData("Target X Error", String.format("%.2f°", redGoalTx));
            }
            
            telemetry.addData("=== TURRET (3-LAYER) ===", "");
            telemetry.addData("Mode", turretMode);
            telemetry.addData("Turret Angle", String.format("%.1f°", turretDeg));
            telemetry.addData("Turret Power", String.format("%.3f", turretPower));
            telemetry.addData("L1 Yaw FF", String.format("%.3f (rate=%.1f°/s)", yawFF, yawRate));
            telemetry.addData("L3 Field Lock", hasFieldLock ?
                    String.format("%.1f° (heading=%.1f°)", lockedFieldAngle, robotHeading) : "NONE");
            telemetry.addData("--- TUNING (Panels) ---", "");
            telemetry.addData("KP/KI/KD", String.format("%.4f / %.5f / %.5f", KP_TURRET, KI_TURRET, KD_TURRET));
            telemetry.addData("K_YAW_FF", String.format("%.4f", K_YAW_FF));
            telemetry.addData("K_FIELD_LOCK", String.format("%.4f", K_FIELD_LOCK));
            
            telemetry.addData("=== SHOOTER ===", "");
            telemetry.addData("Flywheel Velocity", String.format("%.0f", flywheel.getVelocity()));
            sdx.addTelemetry(telemetry);
            
            // === LOOKUP TABLE TEST ===
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
            telemetryM.debug("Red Goal: " + (redGoalVisible ? "YES" : "NO"));
            telemetryM.debug("Turret: " + String.format("%.1f°", turretDeg) + " | Pwr: " + String.format("%.3f", turretPower) + " | " + turretMode);
            telemetryM.debug("L1 YawFF: " + String.format("%.3f", yawFF) + " | Rate: " + String.format("%.1f°/s", yawRate));
            telemetryM.debug("L3 Lock: " + (hasFieldLock ? String.format("%.1f°", lockedFieldAngle) : "NONE") + " | Hdg: " + String.format("%.1f°", robotHeading));
            telemetryM.debug("PID: KP=" + String.format("%.4f", KP_TURRET) + " FF=" + String.format("%.4f", K_YAW_FF));
            telemetryM.debug("Distance: " + (hasDistance ? String.format("%.1f in", distanceInches) : "--"));
            telemetryM.debug("RPM: " + String.format("%.0f", targetRpm) + " | Hood: " + String.format("%.3f", hood.getPosition()));
            telemetryM.debug("Flywheel: " + String.format("%.0f", flywheel.getVelocity()));
            displayTelemetry(this);
            telemetry.update();
            telemetryM.update();
 
        }

    }

    

    // Helper method to convert RPM to ticks per second
    private double getTickSpeed(double rpm) {
        return rpm * 28 / 60; // 28 ticks per revolution, 60 seconds per minute
    }

    /** Normalize any angle to [-180, +180] degrees. */
    private double normalizeAngle(double deg) {
        deg = deg % 360;
        if (deg > 180)  deg -= 360;
        if (deg < -180) deg += 360;
        return deg;
    }

}
