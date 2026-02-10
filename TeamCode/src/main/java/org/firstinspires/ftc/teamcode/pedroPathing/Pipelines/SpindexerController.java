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

    // ========== Slot positions (servo values) ==========
    public static final double P1 = 0.0;
    public static final double P2 = 0.375;
    public static final double P3 = 0.75;
    private static final double[] POSITIONS = {P1, P2, P3};

    // ========== Timing constants ==========
    private static final double SETTLE_SEC    = 0.35;  // wait after servo move before reading sensor
    private static final double CONFIRM_SEC   = 0.1;   // ball must be seen continuously this long
    private static final double HOLD_SEC      = 0.1;   // pause after ball confirmed before rotating

    private static final double SPINUP_SEC    = 1.5;   // flywheel spin-up time
    private static final double FLICK_SEC     = 0.35;  // flicker hold time
    private static final double ADVANCE_SEC   = 0.5;   // wait for spindexer to move between shots

    // ========== Slot tracking ==========
    private String[] slotColor = {"NONE", "NONE", "NONE"};
    private int currentSlot = 0;  // 0=P1, 1=P2, 2=P3

    // ========== Intake ==========
    //  IDLE → SETTLING → SENSING → DETECTED → SETTLING → SENSING → ...
    private enum IState { IDLE, SETTLING, SENSING, DETECTED }
    private IState iState = IState.IDLE;
    private ElapsedTime iTimer = new ElapsedTime();  // shared timer for settle/confirm/hold
    private boolean ballSeen = false;

    // ========== Shoot ==========
    //  IDLE → SPINUP → FIRE → ADVANCE → FIRE → ADVANCE → FIRE → DONE → IDLE
    private enum SState { IDLE, SPINUP, FIRE, ADVANCE, DONE }
    private SState sState = SState.IDLE;
    private ElapsedTime sTimer = new ElapsedTime();
    private int shootSlot = 0;  // which slot we're firing (0, 1, 2)

    // Flicker positions
    private double flickRest1  = 0.1;
    private double flickRest2  = 0.0875;
    private double flickShoot1 = 0.5;
    private double flickShoot2 = 0.5;

    // Flywheel
    private double shootRpm = 3000;

    // ========== Constructor ==========
    public SpindexerController() {}

    public void setFlickerPositions(double r1, double r2, double s1, double s2) {
        flickRest1 = r1; flickRest2 = r2; flickShoot1 = s1; flickShoot2 = s2;
    }

    public void setShootRpm(double rpm) { this.shootRpm = rpm; }

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
                // Servo should already be at the right slot (set at play-start or after last detect).
                // Go straight to settling so we wait for it to be still before reading sensor.
                iState = IState.SETTLING;
                iTimer.reset();
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
                // Ball confirmed — wait HOLD_SEC then rotate to next slot
                if (iTimer.seconds() >= HOLD_SEC) {
                    if (currentSlot < 2) {
                        // Advance to next slot
                        currentSlot++;
                        spindexer.setPosition(POSITIONS[currentSlot]);
                        iState = IState.SETTLING;
                        iTimer.reset();
                    } else {
                        // Already at P3 (last slot) — stay here, keep intake running
                        // Go back to sensing in case the ball gets swapped
                        iState = IState.SENSING;
                    }
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
                    // Start sequence: go to P1, spin up flywheel
                    shootSlot = 0;
                    currentSlot = 0;
                    spindexer.setPosition(POSITIONS[0]);
                    flywheel.setVelocity(rpmToTPS(shootRpm));
                    sTimer.reset();
                    sState = SState.SPINUP;
                }
                break;

            case SPINUP:
                // Wait for flywheel to reach speed
                if (sTimer.seconds() >= SPINUP_SEC) {
                    flicker1.setPosition(flickShoot1);
                    flicker2.setPosition(flickShoot2);
                    sTimer.reset();
                    sState = SState.FIRE;
                }
                break;

            case FIRE:
                // Wait for flick to complete
                if (sTimer.seconds() >= FLICK_SEC) {
                    flicker1.setPosition(flickRest1);
                    flicker2.setPosition(flickRest2);
                    slotColor[shootSlot] = "NONE";

                    shootSlot++;
                    if (shootSlot >= 3) {
                        // All 3 fired — done
                        flywheel.setVelocity(0);
                        sTimer.reset();
                        sState = SState.DONE;
                    } else {
                        // Advance to next slot
                        currentSlot = shootSlot;
                        spindexer.setPosition(POSITIONS[shootSlot]);
                        sTimer.reset();
                        sState = SState.ADVANCE;
                    }
                }
                break;

            case ADVANCE:
                // Wait for spindexer to arrive, then fire
                if (sTimer.seconds() >= ADVANCE_SEC) {
                    flicker1.setPosition(flickShoot1);
                    flicker2.setPosition(flickShoot2);
                    sTimer.reset();
                    sState = SState.FIRE;
                }
                break;

            case DONE:
                if (sTimer.seconds() >= 0.2) {
                    // Reset to P1 so next intake starts fresh
                    currentSlot = 0;
                    spindexer.setPosition(POSITIONS[0]);
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

    /** Move spindexer to a specific slot (for init / manual use) */
    public void goToSlot(int slot) {
        if (slot >= 0 && slot <= 2) {
            currentSlot = slot;
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
        telemetry.addData("Colors", slotColor[0] + " | " + slotColor[1] + " | " + slotColor[2]);
        telemetry.addData("Intake", iState.toString());
        telemetry.addData("Shoot", sState.toString());
    }
}
