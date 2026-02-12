package org.firstinspires.ftc.teamcode.pedroPathing.Pipelines;

import com.qualcomm.robotcore.util.ElapsedTime;

import static org.firstinspires.ftc.teamcode.pedroPathing.Pipelines.Sensor.*;
import static org.firstinspires.ftc.teamcode.pedroPathing.Pipelines.Servo_Pipeline.*;

/**
 * Spindexer Controller — rebuilt from scratch.
 *
 * INTAKE (hold trigger):
 *   - Starts at P1. Intake motor runs entire time trigger is held.
 *   - Sensor checks for ball ONLY while trigger held.
 *   - Ball confirmed → 100ms pause → spindexer rotates to next slot.
 *   - Order: P1 → P2 → P3. After P3 filled, stays there.
 *
 * SHOOT (one press):
 *   - Fires all 3 slots: P1 → P2 → P3.
 *   - Spins up flywheel → flick → advance → flick → advance → flick → done.
 */
public class SpindexerController {

    // ========== Slot positions (servo
    //  values) ==========
    public static final double P1 = 0.02;
    public static final double P2 = 0.375;
    public static final double P3 = 0.75;
    private static final double[] POSITIONS = {P1, P2, P3};

    // ========== Timing constants ==========
    private static final double SETTLE_SEC    = 0.35;  // wait after servo arrives before reading sensor
    private static final double CONFIRM_SEC   = 0.1;   // ball must be seen continuously this long
    private static final double HOLD_SEC      = 0.1;   // pause after ball confirmed before rotating

    private static final double SPINUP_SEC    = 1.5;   // flywheel spin-up time
    private static final double FLICK_SEC     = 0.3;   // flicker hold time (fast snap)
    private static final double FLICK2_DELAY  = 0.02;   // right flicker fires 100ms after left
    private static final double POST_FLICK_SEC = 0.35;  // pause after flicker retracts before advancing

    // ========== Slow servo movement ==========
    private static final double INTAKE_SERVO_STEP = 0.025; // intake speed (~1s per slot at 50Hz)
    private static final double SHOOT_SERVO_STEP  = 0.0125; // shoot speed (~1s per slot at 50Hz)
    private double servoPos  = P1;   // current commanded position
    private double servoTarget = P1; // where we're heading

    // ========== Slot tracking ==========
    private String[] slotColor = {"NONE", "NONE", "NONE"};
    private int currentSlot = 0;  // 0=P1, 1=P2, 2=P3

    // ========== Direction ==========
    // false = P1→P2→P3 (start at P1), true = P3→P2→P1 (start at P3)
    private boolean startFromP3 = false;
    private int direction = 1;  // +1 forward, -1 reverse

    // ========== Intake ==========
    //  IDLE → SETTLING → SENSING → DETECTED → MOVING → SETTLING → SENSING → ...
    private enum IState { IDLE, SETTLING, SENSING, DETECTED, MOVING }
    private IState iState = IState.IDLE;
    private ElapsedTime iTimer = new ElapsedTime();  // shared timer for settle/confirm/hold
    private boolean ballSeen = false;

    // ========== Shoot ==========
    //  IDLE → SPINUP → FIRE → RETRACTING → ADVANCING → FIRE → ... → DONE → IDLE
    private enum SState { IDLE, SPINUP, FIRE, RETRACTING, ADVANCING, DONE }
    private SState sState = SState.IDLE;
    private ElapsedTime sTimer = new ElapsedTime();
    private int shootSlot = 0;  // which slot we're firing (0, 1, 2)

    // Flicker positions
    private double flickRest1  = 0.1;
    private double flickRest2  = 0.1;
    private double flickShoot1 = 0.55;
    private double flickShoot2 = 0.5;

    // Flywheel
    private double shootRpm = 3000;

    // ========== Constructor ==========
    public SpindexerController() {}

    public void setFlickerPositions(double r1, double r2, double s1, double s2) {
        flickRest1 = r1; flickRest2 = r2; flickShoot1 = s1; flickShoot2 = s2;
    }

    public void setShootRpm(double rpm) { this.shootRpm = rpm; }

    /**
     * Set whether intake starts from P3 (reverse order).
     * Call BEFORE play starts. Also resets slot to the start position.
     * @param fromP3  true = P3→P2→P1, false = P1→P2→P3
     */
    public void setStartFromP3(boolean fromP3) {
        this.startFromP3 = fromP3;
        this.direction = fromP3 ? -1 : 1;
        int startSlot = fromP3 ? 2 : 0;
        currentSlot = startSlot;
        servoPos = POSITIONS[startSlot];
        servoTarget = POSITIONS[startSlot];
    }

    /** Get start slot index based on direction */
    private int startSlot() { return startFromP3 ? 2 : 0; }
    /** Get end slot index based on direction */
    private int endSlot()   { return startFromP3 ? 0 : 2; }
    /** Check if we're at the last slot in the fill direction */
    private boolean isLastSlot() { return currentSlot == endSlot(); }
    /** Get next slot in fill direction, or -1 if at end */
    private int nextSlot() {
        int next = currentSlot + direction;
        return (next >= 0 && next <= 2) ? next : -1;
    }

    // ======================================================================
    //  INTAKE — call every loop
    // ======================================================================

    /**
     * @param held  true while driver holds intake trigger
     * @return true if intake motor should run
     */
    public boolean updateIntake(boolean held) {

        // --- Don't process intake while shooting ---
        if (isShooting()) {
            iState = IState.IDLE;
            ballSeen = false;
            return false;
        }

        // --- Trigger released: stop everything, go idle ---
        if (!held) {
            iState = IState.IDLE;
            ballSeen = false;
            return false;
        }

        // --- Trigger held: run the state machine ---
        switch (iState) {

            case IDLE:
                // Just started holding trigger.
                // If servo was interrupted mid-move, finish the move first
                if (Math.abs(servoPos - servoTarget) > 0.01) {
                    iState = IState.MOVING;
                } else {
                    // Already at a slot — settle before reading sensor
                    iState = IState.SETTLING;
                    iTimer.reset();
                }
                return true;  // intake motor on

            case SETTLING:
                // Wait for servo to physically stop before trusting sensor
                if (iTimer.seconds() >= SETTLE_SEC) {
                    iState = IState.SENSING;
                    ballSeen = false;
                }
                return true;  // intake motor on

            case SENSING:
                // Read sensor — look for a ball
                if (isBallPresent()) {
                    if (!ballSeen) {
                        // First frame we see it — start confirmation timer
                        ballSeen = true;
                        iTimer.reset();
                    } else if (iTimer.seconds() >= CONFIRM_SEC) {
                        // Confirmed! Record color, move to DETECTED state
                        slotColor[currentSlot] = detectBallColor();
                        iState = IState.DETECTED;
                        iTimer.reset();
                        ballSeen = false;
                    }
                } else {
                    // Lost it — restart confirmation
                    ballSeen = false;
                }
                return true;  // intake motor on

            case DETECTED:
                // Ball confirmed — wait HOLD_SEC then start moving to next slot
                if (iTimer.seconds() >= HOLD_SEC) {
                    int next = nextSlot();
                    if (next >= 0) {
                        // Start slow move to next slot
                        currentSlot = next;
                        servoTarget = POSITIONS[currentSlot];
                        iState = IState.MOVING;
                    } else {
                        // Already at last slot — stay here, keep intake running
                        iState = IState.SENSING;
                    }
                }
                return true;  // intake motor on

            case MOVING:
                // Slowly step servo toward target position
                if (stepServo(INTAKE_SERVO_STEP)) {
                    // Arrived — now settle before reading sensor
                    iState = IState.SETTLING;
                    iTimer.reset();
                }
                return true;  // intake motor on
        }

        return true;
    }

    // ======================================================================
    //  SHOOT — call every loop
    // ======================================================================

    /**
     * @param shootPressed  true on the frame shoot button is pressed (rising edge)
     * @param killSwitch    true to abort sequence immediately
     * @param flywheel      the flywheel motor (DcMotorEx)
     */
    public void updateShoot(boolean shootPressed, boolean killSwitch,
                            com.qualcomm.robotcore.hardware.DcMotorEx flywheel) {


    
        // Kill switch
        if (killSwitch && sState != SState.IDLE) {
            flywheel.setVelocity(0);
            flicker1.setPosition(flickRest1);
            flicker2.setPosition(flickRest2);
            sState = SState.IDLE;
            return;
        }

        switch (sState) {

            case IDLE:
                if (shootPressed) {
                    // Start sequence: go to start slot, spin up flywheel
                    shootSlot = startSlot();
                    currentSlot = shootSlot;
                    servoTarget = POSITIONS[shootSlot];
                    servoPos = POSITIONS[shootSlot];
                    spindexer.setPosition(POSITIONS[shootSlot]);
                    flywheel.setVelocity(rpmToTPS(shootRpm));
                    sTimer.reset();
                    sState = SState.SPINUP;
                }
                break;

            case SPINUP:
                // Wait for flywheel to reach speed
                if (sTimer.seconds() >= SPINUP_SEC) {
                    flicker1.setPosition(flickShoot1);
                    // flicker2 fires after FLICK2_DELAY in FIRE state
                    sTimer.reset();
                    sState = SState.FIRE;
                }
                break;

            case FIRE:
                // Fire right flicker after delay
                if (sTimer.seconds() >= FLICK2_DELAY) {
                    flicker2.setPosition(flickShoot2);
                }
                // Wait for flick to complete
                if (sTimer.seconds() >= FLICK_SEC) {
                    flicker1.setPosition(flickRest1);
                    flicker2.setPosition(flickRest2);
                    slotColor[shootSlot] = "NONE";

                    int nextShoot = shootSlot + direction;
                    if (nextShoot < 0 || nextShoot > 2) {
                        // All 3 fired — done
                        flywheel.setVelocity(0);
                        sTimer.reset();
                        sState = SState.DONE;
                    } else {
                        // Pause before advancing
                        shootSlot = nextShoot;
                        sTimer.reset();
                        sState = SState.RETRACTING;
                    }
                }
                break;

            case RETRACTING:
                // Brief pause after flicker retracts before moving spindexer
                if (sTimer.seconds() >= POST_FLICK_SEC) {
                    currentSlot = shootSlot;
                    servoTarget = POSITIONS[shootSlot];
                    sState = SState.ADVANCING;
                }
                break;

            case ADVANCING:
                // Slowly step spindexer to next slot, then fire
                if (stepServo(SHOOT_SERVO_STEP)) {
                    // Arrived — small settle then fire
                    flicker1.setPosition(flickShoot1);
                    flicker2.setPosition(flickShoot2);
                    sTimer.reset();
                    sState = SState.FIRE;
                }
                break;

            case DONE:
                if (sTimer.seconds() >= 0.3) {
                    // Reset to start slot so next intake starts fresh
                    int s = startSlot();
                    currentSlot = s;
                    servoPos = POSITIONS[s];
                    servoTarget = POSITIONS[s];
                    spindexer.setPosition(POSITIONS[s]);
                    sState = SState.IDLE;
                }
                break;
        }
    }

    // ======================================================================
    //  UTILITY
    // ======================================================================

    private double rpmToTPS(double rpm) {
        return rpm * 28.0 / 60.0;  // GoBilda 28 ticks/rev
    }

    /**
     * Move servoPos one step toward servoTarget.
     * @param step  position increment per loop (bigger = faster)
     * @return true when arrived (within half a step of target)
     */
    private boolean stepServo(double step) {
        double diff = servoTarget - servoPos;
        if (Math.abs(diff) <= step * 0.5) {
            // Close enough — snap to target
            servoPos = servoTarget;
            spindexer.setPosition(servoPos);
            return true;
        }
        // Step toward target
        servoPos += Math.signum(diff) * step;
        spindexer.setPosition(servoPos);
        return false;
    }

    /** Move spindexer to a specific slot (for init / manual use) */
    public void goToSlot(int slot) {
        if (slot >= 0 && slot <= 2) {
            currentSlot = slot;
            servoPos = POSITIONS[slot];
            servoTarget = POSITIONS[slot];
            spindexer.setPosition(POSITIONS[slot]);
        }
    }

    // ========== Getters ==========
    public int     getCurrentSlot()   { return currentSlot; }
    public String  getSlotColor(int i){ return (i >= 0 && i < 3) ? slotColor[i] : "NONE"; }
    public boolean isShooting()       { return sState != SState.IDLE && sState != SState.DONE; }

    /** Add telemetry */
    public void addTelemetry(org.firstinspires.ftc.robotcore.external.Telemetry telemetry) {
        telemetry.addData("--- Spindexer ---", "");
        telemetry.addData("Slot", "P" + (currentSlot + 1));
        telemetry.addData("Direction", startFromP3 ? "P3→P1" : "P1→P3");
        telemetry.addData("Colors", slotColor[0] + " | " + slotColor[1] + " | " + slotColor[2]);
        telemetry.addData("Intake State", iState.toString());
        telemetry.addData("Shoot State", sState.toString());
        telemetry.addData("Servo Pos/Target", String.format("%.3f / %.3f", servoPos, servoTarget));
        telemetry.addData("Spindexer Pos", String.format("%.3f", spindexer.getPosition()));
        telemetry.addData("Ball Present?", isBallPresent() ? "YES" : "NO");
        // Raw sensor data for debugging
        com.qualcomm.robotcore.hardware.NormalizedRGBA c = ballSensor.getNormalizedColors();
        float brightness = Math.max(c.red, Math.max(c.green, c.blue));
        float hue = rgbToHue(c.red, c.green, c.blue);
        telemetry.addData("Sensor", String.format("H:%.0f B:%.2f  R:%.2f G:%.2f B:%.2f", hue, brightness, c.red, c.green, c.blue));
    }
}
