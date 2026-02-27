package org.firstinspires.ftc.teamcode.pedroPathing.TeleOp;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import java.util.List;

import static org.firstinspires.ftc.teamcode.pedroPathing.Pipelines.Motor_PipeLine.*;
import static org.firstinspires.ftc.teamcode.pedroPathing.Pipelines.Servo_Pipeline.*;
import static org.firstinspires.ftc.teamcode.pedroPathing.Pipelines.Limelight_Pipeline.*;
import org.firstinspires.ftc.teamcode.pedroPathing.Pipelines.Limelight_Pipeline;
import static org.firstinspires.ftc.teamcode.pedroPathing.Pipelines.Sensor.*;
import org.firstinspires.ftc.teamcode.pedroPathing.Pipelines.SpindexerController;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.robotcore.internal.camera.calibration.CameraCalibration;
import org.firstinspires.ftc.robotcore.external.stream.CameraStreamServer;
import android.util.Size;



@TeleOp(name="Cassius Blue", group="TeleOp")
public class Cassius_Blue extends LinearOpMode {



    @Override 
    public void runOpMode() throws InterruptedException {

        // pipelines 
        intMotors(this);
        intServos(this);
        initLimelight(this);
        initSensors(this);

        // Direct Limelight reference for turret PID (bypasses pipeline abstraction)
        Limelight3A limelight = hardwareMap.get(Limelight3A.class, "limelight");

        // Open Logitech webcam via VisionPortal so Panels CameraStream widget can display it
        VisionPortal cameraPortal = new VisionPortal.Builder()
                .setCamera(hardwareMap.get(WebcamName.class, "logitech"))
                .setCameraResolution(new Size(640, 480))
                .build();

        // Route this portal to the DS / Panels camera stream
        CameraStreamServer.getInstance().setSource(cameraPortal);

        telemetry.addData("Status", "Hardware initialized");
        telemetry.update();


        // put all initlization code here

        double gear = 1.25; // speed modifier for drive train
        
        // Spindexer controller — handles intake detection, slot tracking, and shooting
        SpindexerController sdx = new SpindexerController();
        sdx.setFlickerPositions(0, 0.375);
        flicker.setPosition(0);


       



    double baseRpm = 3000; // default RPM for shooter (NOTE: 'intake' variable is actually the shooter motor)
    double targetRpm = baseRpm; // updated from Limelight lookup when available
    double targetHoodPos = hood.getPosition();
    // Limelight distance config (inches/degrees)
    double cameraHeight = 11.125; // Camera lens center height in inches
    double cameraMountAngle = 22.85; // Camera angle from horizontal (calibrated)
    double targetHeight = 29.5; // AprilTag center height in inches

    // Smoothing filter for distance — prevents hood jitter from noisy Limelight frames
    double smoothedDistance = -1; // -1 means no value yet
    double SMOOTHING_ALPHA = 0.3; // 0.0 = very smooth/slow, 1.0 = no smoothing (raw)
      
        // Turret safety limits in DEGREES
        double turretMinDeg = 5;
        double turretMaxDeg = 280;
        boolean limitsEnabled = true;
        double LIMIT_SLOW_ZONE_DEG = 15;
        double TICKS_PER_DEG = 2.64;

        // Mag sensor = turret home (position 0)
        boolean lastMagState = false;

        // === TURRET PID (from Turret_try — direct Limelight, no IMU) ===
        double kP = 0.015;
        double kI = 0.0001;
        double kD = 0.002;
        double targetX = 0.0;
        double turretIntegral = 0.0;
        double turretLastError = 0.0;
        ElapsedTime turretTimer = new ElapsedTime();
        ElapsedTime targetLostTimer = new ElapsedTime();

        double POSITION_TOLERANCE = 1.5;
        double MIN_POWER = 0.05;
        double MAX_TURRET_POWER = 0.4;
        double TARGET_LOST_TIMEOUT = 2.0;
        double HOME_POWER = -0.2;
        boolean INVERT_MOTOR = false;

        boolean targetWasVisible = false;
        boolean homingToMag = false;
        double lastTurretPower = 0; // remember last power for coasting (matches Turret_try)
        boolean autoAimEnabled = true;  // Y2 toggles this
        boolean lastY2 = false;         // edge detection for Y2

        // === MOTIF FIRE: left trigger starts sequence ===
        // Sequence: turn turret → 20°, lock on via Limelight, auto-shoot
        int motifState = 0; // 0=IDLE, 1=TURN_TO_20, 2=LOCK_ON, 3=SHOOT
        ElapsedTime motifTimer = new ElapsedTime();
        boolean lastLTrigger1 = false; // edge detection for left trigger
        double MOTIF_TARGET_DEG = 20.0;
        boolean motifShootTriggered = false;

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
            turret.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            homed = true;
        }
        while (!homed && !isStarted() && !isStopRequested()) {
            updateSensors();
            if (isMagPressed()) {
                turret.setPower(0);
                turret.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                turret.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
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
        spindexerAxon.setTargetRotation(SpindexerController.P1); // start at P1 once
        while (opModeIsActive()) {
               // Poll Limelight ONCE per loop — all getters below see the SAME frame
               pollOnce();
               // Update sensors every loop so isMagPressed() stays current
               updateSensors();
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


            
            telemetry.addData("spindexer angle", String.format("%.1f\u00b0", spindexerAxon.getRawAngle()));
            // Manual turret dpad removed — auto-aim handles turret now


            // Intake with automatic spindexer rotation via SpindexerController
            boolean intakeRequested = RTrigger1 > 0.1;
            boolean runIntakeMotor = sdx.updateIntake(intakeRequested);

            if (runIntakeMotor) {
                intake.setPower(1); // intake in (spindexer auto-rotates on ball detect)
                intake2.setPower(1);
            } else if (sdx.isShooting()) {
                intake.setPower(0); // stop intake during shoot sequence
                intake2.setPower(0);
            } else if (RBumper1) {
                intake.setPower(-0.6); // intake out (reverse)
                intake2.setPower(-0.6);
            } else {
                intake.setPower(0);
                intake2.setPower(0);
            }


            // Limelight-based shooter tuning (distance -> rpm + hood)
            // Only use blue goal (AprilTag ID 20) for distance/hood
            LLResult limelightResult = getLatestResult();
            double distanceInches = 0.0;
            boolean hasDistance = false;
            if (limelightResult != null && limelightResult.isValid() &&
                limelightResult.getFiducialResults() != null && !limelightResult.getFiducialResults().isEmpty()) {
                // Find blue goal tag (ID 20) specifically
                LLResultTypes.FiducialResult blueGoalForHood = null;
                for (LLResultTypes.FiducialResult f : limelightResult.getFiducialResults()) {
                    if ((int) f.getFiducialId() == 20) {
                        blueGoalForHood = f;
                        break;
                    }
                }
                if (blueGoalForHood != null) {
                    // --- 3D pose distance (more accurate than trig) ---
                    Pose3D tagPose = blueGoalForHood.getTargetPoseCameraSpace();
                    if (tagPose != null) {
                        double xMeters = tagPose.getPosition().x;
                        double zMeters = tagPose.getPosition().z;
                        distanceInches = Math.sqrt(xMeters * xMeters + zMeters * zMeters) * 39.3701;
                        hasDistance = true;
                    } else {
                        // Fallback to trig if 3D pose unavailable
                        double ty = blueGoalForHood.getTargetYDegrees();
                        double totalAngle = cameraMountAngle + ty;
                        double heightDifference = targetHeight - cameraHeight;
                        if (Math.abs(totalAngle) > 0.5 && Math.abs(totalAngle) < 89.5) {
                            distanceInches = heightDifference / Math.tan(Math.toRadians(totalAngle));
                            hasDistance = true;
                        }
                    }
                }
            }
            if (hasDistance) {
                // Smooth the distance to eliminate frame-to-frame jitter
                if (smoothedDistance < 0) {
                    smoothedDistance = distanceInches; // first reading — use raw
                } else {
                    smoothedDistance = smoothedDistance + SMOOTHING_ALPHA * (distanceInches - smoothedDistance);
                }
                // Lookup table is primary — always applies when target is visible
                ShooterLookup.Result tuned = ShooterLookup.lookup(smoothedDistance);
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
            //   Motif fire state 3 also triggers a shoot
            boolean motifShootNow = (motifState == 3 && !motifShootTriggered);
            sdx.updateShoot(RBumper2 || motifShootNow, LBumper2, flywheel);
            if (motifShootNow) motifShootTriggered = true;
            // Motif fire: finish once shoot sequence completes
            if (motifState == 3 && motifShootTriggered && !sdx.isShooting()) {
                motifState = 0; // sequence complete → back to IDLE
            }

            // Mag sensor state tracking
            boolean currentMagState = isMagPressed();
            lastMagState = currentMagState;

            // === MOTIF FIRE: left trigger starts the sequence ===
            boolean ltPressed = LTrigger1 > 0.25;
            if (ltPressed && !lastLTrigger1 && motifState == 0) {
                motifState = 1;  // start → turn turret to 20°
                motifTimer.reset();
                motifShootTriggered = false;
            }
            lastLTrigger1 = ltPressed;

            // ================================================================
            //  TURRET TRACKING (from Turret_try — direct Limelight PID)
            //  Only tracks blue goal (AprilTag ID 20)
            // ================================================================

            int turretPosition = turret.getCurrentPosition();
            double turretDeg = turretPosition / TICKS_PER_DEG;
            double turretPower = 0;
            String turretMode = "IDLE";

            // --- Y2 toggle: auto-aim on/off ---
            if (y2 && !lastY2) autoAimEnabled = !autoAimEnabled;
            lastY2 = y2;

            // --- Manual turret control — gamepad2 dpad left/right ---
            boolean manualTurret = dpadLeft2 || dpadRight2;

            if (motifState == 1) {
                // === MOTIF FIRE step 1: turn turret to 20° ===
                double motifError = MOTIF_TARGET_DEG - turretDeg;
                turretPower = Math.max(-0.3, Math.min(0.3, motifError * 0.02));
                turretMode = "MOTIF\u219220\u00b0";
                if (Math.abs(motifError) < 2.0) {
                    motifState = 2;  // close enough → start locking on
                    motifTimer.reset();
                }
            } else if (manualTurret) {
                // === MANUAL OVERRIDE ===
                turretPower = (dpadRight2 ? 1 : -1) * 0.4;
                targetWasVisible = false;
                homingToMag = false;
                turretIntegral = 0;
                turretLastError = 0;
                turretMode = "MANUAL";
                if (motifState > 0) motifState = 0; // cancel motif on manual override
            } else if (!autoAimEnabled) {
                // Auto-aim disabled — hold position
                turretPower = 0;
                turretMode = "AIM OFF";
            } else {
                // === AUTO-AIM: read Limelight directly, filter for blue goal (tag 20) ===
                try {
                    LLResult result = limelight.getLatestResult();

                    LLResultTypes.FiducialResult blueGoal = null;
                    if (result != null && result.isValid()) {
                        List<LLResultTypes.FiducialResult> fiducials = result.getFiducialResults();
                        if (fiducials != null) {
                            for (LLResultTypes.FiducialResult f : fiducials) {
                                if ((int) f.getFiducialId() == 20) {
                                    blueGoal = f;
                                    break;
                                }
                            }
                        }
                    }

                    if (blueGoal != null) {
                        // Blue goal found — PID tracking
                        targetWasVisible = true;
                        targetLostTimer.reset();
                        homingToMag = false;
                        turretMode = "LOCKED ON";

                        double tx = blueGoal.getTargetXDegrees();

                        double dt = turretTimer.seconds();
                        turretTimer.reset();
                        if (dt < 0.001) dt = 0.001;
                        if (dt > 1.0) dt = 1.0;

                        double error = tx - targetX;

                        turretIntegral += error * dt;
                        turretIntegral = Math.max(-50, Math.min(50, turretIntegral));

                        double derivative = (error - turretLastError) / dt;

                        double pidOutput = (kP * error) + (kI * turretIntegral) + (kD * derivative);

                        if (Math.abs(error) < POSITION_TOLERANCE) {
                            pidOutput = 0;
                            turretIntegral = 0;
                        }

                        if (pidOutput != 0 && Math.abs(pidOutput) < MIN_POWER) {
                            pidOutput = MIN_POWER * Math.signum(pidOutput);
                        }

                        turretPower = Math.max(-MAX_TURRET_POWER, Math.min(MAX_TURRET_POWER, pidOutput));
                        if (INVERT_MOTOR) turretPower = -turretPower;
                        if (!Double.isFinite(turretPower)) {
                            turretPower = 0;
                            turretIntegral = 0;
                            turretLastError = 0;
                        }

                        turretLastError = error;

                    } else {
                        // No blue goal visible — freeze turret in place
                        turretPower = 0;
                        turretIntegral = 0;
                        turretLastError = 0;
                        turretMode = "HOLDING";
                    }
                } catch (Exception e) {
                    turretPower = 0;
                    turretIntegral = 0;
                    turretLastError = 0;
                    targetWasVisible = false;
                    homingToMag = false;
                    turretMode = "ERROR";
                    telemetry.addData("Turret Error", e.getMessage());
                }
            }

            // ================================================================
            //  HARD LIMIT ENFORCEMENT — turret CANNOT move past limits
            // ================================================================
            if (limitsEnabled) {
                if (turretDeg >= turretMaxDeg && turretPower > 0) turretPower = 0;
                if (turretDeg <= turretMinDeg && turretPower < 0) turretPower = 0;

                if (turretPower > 0 && turretDeg > turretMaxDeg - LIMIT_SLOW_ZONE_DEG) {
                    double s = (turretMaxDeg - turretDeg) / LIMIT_SLOW_ZONE_DEG;
                    turretPower *= Math.max(0.0, Math.min(1.0, s));
                }
                if (turretPower < 0 && turretDeg < turretMinDeg + LIMIT_SLOW_ZONE_DEG) {
                    double s = (turretDeg - turretMinDeg) / LIMIT_SLOW_ZONE_DEG;
                    turretPower *= Math.max(0.0, Math.min(1.0, s));
                }

                if (turretDeg > turretMaxDeg) turretPower = -0.15;
                if (turretDeg < turretMinDeg) turretPower = 0.15;
            }

            turret.setPower(turretPower);
            lastTurretPower = turretPower; // save for coasting next cycle

            // Motif fire: state 2 — wait for Limelight lock, then transition to shoot
            if (motifState == 2) {
                if (turretMode.equals("LOCKED ON") || motifTimer.seconds() > 3.0) {
                    motifState = 3;  // locked on (or timeout) → shoot!
                    motifTimer.reset();
                    motifShootTriggered = false;
                }
            }

            // Display telemetry
            telemetry.addData("=== LIMELIGHT ===", "");
            if (hasTarget()) {
                LLResult llCheck = getLatestResult();
                if (llCheck != null && llCheck.isValid() && llCheck.getFiducialResults() != null && !llCheck.getFiducialResults().isEmpty()) {
                    int firstTag = (int) llCheck.getFiducialResults().get(0).getFiducialId();
                    telemetry.addData("Tag ID", firstTag);
                } else {
                    telemetry.addData("Tag ID", "NONE");
                }
            } else {
                telemetry.addData("Tag ID", "NONE");
            }
            
            telemetry.addData("=== TURRET ===", "");
            telemetry.addData("Mode", turretMode);
            telemetry.addData("Turret Angle", String.format("%.1f°", turretDeg));
            String[] motifLabels = {"IDLE", "TURN→20°", "LOCKING", "SHOOTING"};
            telemetry.addData("Motif Fire", motifLabels[motifState]);
            
            telemetry.addData("=== SHOOTER ===", "");
            telemetry.addData("Target RPM", String.format("%.0f", targetRpm));
            telemetry.addData("Actual RPM", String.format("%.0f", flywheel.getVelocity() * 60.0 / 28.0));
            telemetry.addData("Hood Angle", String.format("%.3f", hood.getPosition()));
            telemetry.addData(">>>  SENSOR COLOR  <<<", isBallPresent() ? detectBallColor() : "-- empty --");
            sdx.addTelemetry(telemetry);
            

            
            // Push key data to Panels
            telemetryM.debug("Turret: " + String.format("%.1f\u00b0", turretDeg) + " | Pwr: " + String.format("%.3f", turretPower) + " | " + turretMode);
            telemetryM.debug("PID: kP=" + String.format("%.4f", kP) + " kI=" + String.format("%.5f", kI) + " kD=" + String.format("%.4f", kD));
            telemetryM.debug("Mag: " + (isMagPressed() ? "PRESSED" : "---"));
            telemetryM.debug("Distance: " + (hasDistance ? String.format("%.1f in", distanceInches) : "--"));
            telemetryM.debug("RPM: " + String.format("%.0f", targetRpm) + " | Hood: " + String.format("%.3f", hood.getPosition()));
            telemetryM.debug("Flywheel: " + String.format("%.0f", flywheel.getVelocity()));
            telemetry.update(); // Push telemetry to Driver Station
            telemetryM.update(); // Push Panels data separately
 
        }

    }

    

    // Helper method to convert RPM to ticks per second
    private double getTickSpeed(double rpm) {
        return rpm * 28 / 60; // 28 ticks per revolution, 60 seconds per minute
    }

}

