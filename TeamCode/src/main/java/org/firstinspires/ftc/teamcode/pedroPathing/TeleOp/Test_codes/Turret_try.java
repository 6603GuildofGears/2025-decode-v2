
package org.firstinspires.ftc.teamcode.pedroPathing.TeleOp.Test_codes;


import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import java.util.List;

import static org.firstinspires.ftc.teamcode.pedroPathing.Pipelines.Motor_PipeLine.*;
import static org.firstinspires.ftc.teamcode.pedroPathing.Pipelines.Sensor.*;
@Disabled
@TeleOp(name = "Auto-aiming turret", group = "Iterative Opmode")
public class Turret_try extends OpMode {

    // Hardware
    private DcMotorEx turretMotor;
    private Limelight3A limelight;

    // PID Controller variables
    private double kP = 0.015; // Proportional gain - START LOW and increase
    private double kI = 0.0001; // Integral gain - usually keep small
    private double kD = 0.002; // Derivative gain - helps reduce overshoot

    private double targetX = 0.0; // Target is centered (tx = 0)
    private double integral = 0.0;
    private double lastError = 0.0;
    private ElapsedTime timer = new ElapsedTime();
    private ElapsedTime targetLostTimer = new ElapsedTime();

    // Deadband and limits
    private static final double POSITION_TOLERANCE = 1.5; // degrees
    private static final double MIN_POWER = 0.05; // Minimum power to overcome friction
    private static final double MAX_POWER = 0.4; // Maximum turret speed
    private static final double TARGET_LOST_TIMEOUT = 2.0; // seconds before homing to mag sensor

    // Homing speed toward mag sensor
    private static final double HOME_POWER = -0.2; // negative = toward mag sensor (left)

    // Direction tuning - CHANGE THIS IF TURRET MOVES WRONG WAY
    private static final boolean INVERT_MOTOR = false; // Set to true if it moves backwards

    private boolean targetWasVisible = false;
    private boolean homingToMag = false;

    double gear = 1.25;

    @Override
    public void init() {
        try {
            intMotors(this);
            initSensors(this);

            turretMotor = hardwareMap.get(DcMotorEx.class, "turret");
            turretMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

            limelight = hardwareMap.get(Limelight3A.class, "limelight");
            limelight.pipelineSwitch(0); // Use your AprilTag pipeline
            limelight.start();

            timer.reset();
            targetLostTimer.reset();

            telemetry.addData("Status", "Initialized");
            telemetry.addData("Motor Inverted", INVERT_MOTOR);
        } catch (Exception e) {
            telemetry.addData("Init Error", e.getMessage());
        }
    }

    @Override
    public void loop() {

        // Update sensors each loop
        updateSensors();

        // === Gamepad Inputs ===
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

        // === Drive Code ===
        if (Math.abs(LStickX) > 0 || Math.abs(LStickY) > 0 || Math.abs(RStickX) > 0) {
            double rotation = 0;

            double r = Math.hypot(LStickX, LStickY);
            double robotAngle = Math.atan2(LStickY, LStickX) - Math.PI / 4;
            double rightX = RStickX;

            double v1 = r * Math.cos(robotAngle) + rightX * gear; //lf
            double v2 = r * Math.sin(robotAngle) - rightX * gear; //rf
            double v3 = r * Math.sin(robotAngle) + rightX * gear; //lb
            double v4 = r * Math.cos(robotAngle) - rightX * gear; //rb

            SetPower(v1, v3, v2, v4);

        } else if (LBumper1) {
            SetPower(gear, -gear, gear, -gear);

        } else if (LTrigger1 > 0.25) {
            SetPower(gear, -gear, gear, -gear);

        } else if (dpadUp1) {
            SetPower(1, 1, 1, 1);
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

        // === Turret Auto-Aim ===
        try {
            LLResult result = limelight.getLatestResult();

            // Check if we have a valid result with fiducial data
            if (result != null && result.isValid()) {
                List<LLResultTypes.FiducialResult> fiducials =
                    result.getFiducialResults();

                // Find the blue goal tag (ID 20) specifically
                LLResultTypes.FiducialResult blueGoal = null;
                if (fiducials != null) {
                    for (LLResultTypes.FiducialResult f : fiducials) {
                        if ((int) f.getFiducialId() == 20) {
                            blueGoal = f;
                            break;
                        }
                    }
                }

                if (blueGoal != null) {
                    // Blue goal tag found!
                    targetWasVisible = true;
                    targetLostTimer.reset();
                    homingToMag = false;

                    // Get the blue goal fiducial
                    LLResultTypes.FiducialResult fiducial = blueGoal;

                    // Get horizontal offset from center (tx)
                    double tx = fiducial.getTargetXDegrees();

                    // Calculate time since last update
                    double dt = timer.seconds();
                    timer.reset();

                    // Prevent division by zero or huge derivatives
                    if (dt < 0.001) dt = 0.001;
                    if (dt > 1.0) dt = 1.0; // Cap dt if loop was slow

                    // Calculate error
                    double error = tx - targetX;

                    // PID calculations
                    integral += error * dt;

                    // Anti-windup: prevent integral from getting too large
                    integral = Math.max(-50, Math.min(50, integral));

                    double derivative = (error - lastError) / dt;

                    // Calculate PID output
                    double pidOutput =
                        (kP * error) + (kI * integral) + (kD * derivative);

                    // Apply deadband - stop if close enough
                    if (Math.abs(error) < POSITION_TOLERANCE) {
                        pidOutput = 0;
                        integral = 0; // Reset integral when on target
                    }

                    // Apply minimum power if not zero (overcome static friction)
                    if (pidOutput != 0) {
                        if (Math.abs(pidOutput) < MIN_POWER) {
                            pidOutput = MIN_POWER * Math.signum(pidOutput);
                        }
                    }

                    // Clamp output to max power
                    double motorPower = Math.max(
                        -MAX_POWER,
                        Math.min(MAX_POWER, pidOutput)
                    );

                    // Apply inversion if needed
                    if (INVERT_MOTOR) {
                        motorPower = -motorPower;
                    }

                    // Safety check: ensure motor power is finite
                    if (!Double.isFinite(motorPower)) {
                        motorPower = 0;
                        resetPID();
                    }

                    turretMotor.setPower(motorPower);

                    lastError = error;

                    // Telemetry for tuning
                    telemetry.addData("Status", "LOCKED ON");
                    telemetry.addData("TX (degrees)", "%.2f", tx);
                    telemetry.addData(
                        "Target Position",
                        tx > 0 ? "RIGHT" : (tx < 0 ? "LEFT" : "CENTER")
                    );
                    telemetry.addData("Error", "%.2f", error);
                    telemetry.addData("PID Output", "%.3f", pidOutput);
                    telemetry.addData("Motor Power", "%.3f", motorPower);
                    telemetry.addData(
                        "Motor Direction",
                        motorPower > 0
                            ? "POSITIVE"
                            : (motorPower < 0 ? "NEGATIVE" : "STOPPED")
                    );
                    telemetry.addData("Tag ID", fiducial.getFiducialId());
                    telemetry.addData("# Tags Detected", fiducials.size());
                } else {
                    // Result valid but no fiducials detected
                    handleNoTarget();
                }
            } else {
                // No valid result
                handleNoTarget();
            }
        } catch (Exception e) {
            // Catch any errors to prevent crash
            telemetry.addData("Error", e.getMessage());
            telemetry.addData("Error Type", e.getClass().getSimpleName());
            stopTurret();
        }

        telemetry.update();
    }

    private void handleNoTarget() {
        // Target lost - decide what to do
        if (
            targetWasVisible && targetLostTimer.seconds() < TARGET_LOST_TIMEOUT
        ) {
            // Recently lost target, maintain last power briefly (coast)
            telemetry.addData("Status", "TRACKING LOST - Coasting");
            telemetry.addData("Time Lost", "%.2f s", targetLostTimer.seconds());
        } else {
            // Lost for more than 2 seconds - drive turret to mag sensor (home)
            homingToMag = true;
            if (isMagPressed()) {
                // Arrived at home position
                turretMotor.setPower(0);
                resetPID();
                telemetry.addData("Status", "HOME (mag sensor)");
            } else {
                // Drive toward mag sensor
                turretMotor.setPower(HOME_POWER);
                telemetry.addData("Status", "HOMING TO MAG SENSOR");
            }
            telemetry.addData("Mag Sensor", isMagPressed() ? "PRESSED" : "not pressed");
        }
    }

    private void stopTurret() {
        turretMotor.setPower(0);
        resetPID();
        targetWasVisible = false;
        homingToMag = false;
    }

    private void resetPID() {
        integral = 0;
        lastError = 0;
    }

    @Override
    public void stop() {
        try {
            if (turretMotor != null) {
                turretMotor.setPower(0);
            }
            if (limelight != null) {
                limelight.stop();
            }
        } catch (Exception e) {
            // Fail silently on stop
        }
    }
}
