
package org.firstinspires.ftc.teamcode.pedroPathing.TeleOp.Test_codes;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import java.util.List;

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
    private static final double TARGET_LOST_TIMEOUT = 0.5; // seconds before resetting

    // Direction tuning - CHANGE THIS IF TURRET MOVES WRONG WAY
    private static final boolean INVERT_MOTOR = false; // Set to true if it moves backwards

    private boolean targetWasVisible = false;

    @Override
    public void init() {
        try {
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
        try {
            LLResult result = limelight.getLatestResult();

            // Check if we have a valid result with fiducial data
            if (result != null && result.isValid()) {
                List<LLResultTypes.FiducialResult> fiducials =
                    result.getFiducialResults();

                if (fiducials != null && !fiducials.isEmpty()) {
                    // Target found!
                    targetWasVisible = true;
                    targetLostTimer.reset();

                    // Get the first (closest/best) fiducial
                    LLResultTypes.FiducialResult fiducial = fiducials.get(0);

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
            // Target lost for too long - stop and reset
            stopTurret();
            telemetry.addData("Status", "SEARCHING");
            telemetry.addData("Target Visible", "No");
        }
    }

    private void stopTurret() {
        turretMotor.setPower(0);
        resetPID();
        targetWasVisible = false;
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
