package org.firstinspires.ftc.teamcode.pedroPathing.Pipelines;

import com.qualcomm.robotcore.util.ElapsedTime;

import static org.firstinspires.ftc.teamcode.pedroPathing.Pipelines.Sensor.*;
import static org.firstinspires.ftc.teamcode.pedroPathing.Pipelines.Servo_Pipeline.*;

/**
 * Spindexer Controller — manages ball intake, tracking, and shooting
 * for the walled/partitioned spindexer system.
 *
 * 3 slots (p1, p2, p3) rotate under a single active position where both
 * the intake and shooter are located. A REV Color Sensor V3 ("ballSensor")
 * sits at that position to detect ball presence and color.
 *
 * INTAKE: Starts from p1 or p3 (edges first for efficiency).
 *   - Color sensor detects ball arrival → logs color → rotates to next empty slot
 *   - Stops when all 3 slots are full
 *
 * SHOOT: Fires loaded slots in order, skipping empties.
 *   - Rotates loaded slot into active position → fires flickers → next loaded slot
 *   - Tracks which slots have been emptied
 *
 * Usage:
 *   SpindexerController sdx = new SpindexerController();
 *   // In loop:
 *   sdx.updateIntake(intakeButtonHeld);
 *   sdx.updateShoot(shootRequested, killSwitch, flywheel, flicker1, flicker2, ...);
 *   sdx.addTelemetry(telemetry);
 */
public class SpindexerController {

    // ========== Slot positions (servo values) ==========
    public static final double P1 = 0.0;
    public static final double P2 = 0.375;
    public static final double P3 = 0.75;
    private static final double[] SLOT_POSITIONS = {P1, P2, P3};

    // ========== Ball detection ==========
    private static final double BALL_PRESENCE_THRESHOLD = 0.15; // alpha threshold (tune on real hardware)
    private static final double SETTLE_TIME_SEC = 0.25;         // wait for spindexer to physically arrive

    // ========== Slot tracking ==========
    private boolean[] slotLoaded = {false, false, false};  // true = ball in slot
    private String[]  slotColor  = {"NONE", "NONE", "NONE"}; // detected color per slot
    private int       currentSlot = 0;   // which slot (0/1/2) is at the active position
    private int       ballCount   = 0;   // how many balls loaded

    // ========== Intake state ==========
    public enum IntakeState { IDLE, WAITING_FOR_BALL, SETTLING, FULL }
    private IntakeState intakeState = IntakeState.IDLE;
    private boolean intakeActive = false;
    private ElapsedTime settleTimer = new ElapsedTime();

    // Intake order: edges first (p1=0, p3=2, p2=1) for efficiency
    private static final int[] INTAKE_ORDER = {0, 2, 1};
    private int intakeOrderIndex = 0;

    // ========== Shoot state ==========
    public enum ShootState { IDLE, SPINNING_UP, FIRING, ADVANCING, DONE }
    private ShootState shootState = ShootState.IDLE;
    private ElapsedTime shootTimer = new ElapsedTime();
    private int shootSlotIndex = -1; // index into the loaded-slots list
    private int[] shootOrder;        // dynamically built from loaded slots
    private int shootOrderLen = 0;

    // Shooting timing constants
    private static final double SPINUP_TIME_SEC   = 1.5;  // flywheel spin-up
    private static final double FLICK_TIME_SEC    = 0.35; // time to hold flicker out
    private static final double ADVANCE_TIME_SEC  = 0.35; // time for spindexer to rotate to next

    // Flicker positions
    private double flickerRestPos1  = 0.1;
    private double flickerRestPos2  = 0.0875;
    private double flickerShootPos1 = 0.5;
    private double flickerShootPos2 = 0.5;

    // Flywheel RPM
    private double shootRpm = 3000;

    // ========== Constructor ==========
    public SpindexerController() {
        reset();
    }

    /**
     * Set flicker positions (call once during init if different from defaults)
     */
    public void setFlickerPositions(double rest1, double rest2, double shoot1, double shoot2) {
        flickerRestPos1 = rest1;
        flickerRestPos2 = rest2;
        flickerShootPos1 = shoot1;
        flickerShootPos2 = shoot2;
    }

    /**
     * Set shoot RPM (can be updated from Limelight distance lookup)
     */
    public void setShootRpm(double rpm) {
        this.shootRpm = rpm;
    }

    // ==========================================================================
    //  INTAKE LOGIC
    // ==========================================================================

    /**
     * Call every loop while intake button is held.
     * Automatically detects balls and rotates to next empty slot.
     *
     * @param intakeHeld  true while the driver holds the intake button
     * @return true if intake motor should run (pass-through to motor control)
     */
    public boolean updateIntake(boolean intakeHeld) {
        if (!intakeHeld) {
            if (intakeState != IntakeState.FULL) {
                intakeState = IntakeState.IDLE;
            }
            intakeActive = false;
            return false;
        }

        // Already full — don't intake
        if (ballCount >= 3) {
            intakeState = IntakeState.FULL;
            intakeActive = false;
            return false;
        }

        switch (intakeState) {
            case IDLE:
                // Start intake: go to first empty slot
                intakeOrderIndex = 0;
                if (goToNextEmptySlot()) {
                    intakeState = IntakeState.SETTLING;
                    settleTimer.reset();
                } else {
                    intakeState = IntakeState.FULL;
                }
                break;

            case SETTLING:
                // Wait for spindexer to physically arrive at position
                if (settleTimer.seconds() >= SETTLE_TIME_SEC) {
                    intakeState = IntakeState.WAITING_FOR_BALL;
                }
                intakeActive = true;
                return true; // run intake motor while settling too

            case WAITING_FOR_BALL:
                intakeActive = true;
                // Check color sensor for ball arrival
                if (isBallPresent(BALL_PRESENCE_THRESHOLD)) {
                    // Ball detected! Log it
                    slotLoaded[currentSlot] = true;
                    slotColor[currentSlot] = detectBallColor(BALL_PRESENCE_THRESHOLD);
                    ballCount++;

                    if (ballCount >= 3) {
                        intakeState = IntakeState.FULL;
                        intakeActive = false;
                        return false;
                    }

                    // Advance to next empty slot
                    intakeOrderIndex++;
                    if (goToNextEmptySlot()) {
                        intakeState = IntakeState.SETTLING;
                        settleTimer.reset();
                    } else {
                        intakeState = IntakeState.FULL;
                        intakeActive = false;
                        return false;
                    }
                }
                return true; // keep intake motor running

            case FULL:
                intakeActive = false;
                return false;
        }

        return intakeActive;
    }

    /**
     * Find the next empty slot in intake order and move spindexer there.
     * @return true if an empty slot was found, false if all full
     */
    private boolean goToNextEmptySlot() {
        for (int i = intakeOrderIndex; i < INTAKE_ORDER.length; i++) {
            int slot = INTAKE_ORDER[i];
            if (!slotLoaded[slot]) {
                currentSlot = slot;
                spindexer.setPosition(SLOT_POSITIONS[slot]);
                intakeOrderIndex = i;
                return true;
            }
        }
        return false; // all slots full
    }

    // ==========================================================================
    //  SHOOT LOGIC
    // ==========================================================================

    /**
     * Call every loop to manage the shooting sequence.
     *
     * @param shootRequested  true on the frame the driver presses shoot (rising edge)
     * @param killSwitch      true to immediately abort shooting
     * @param flywheel        the flywheel motor (DcMotorEx with velocity control)
     */
    public void updateShoot(boolean shootRequested, boolean killSwitch,
                            com.qualcomm.robotcore.hardware.DcMotorEx flywheel) {

        // Kill switch — emergency stop
        if (killSwitch && shootState != ShootState.IDLE) {
            flywheel.setVelocity(0);
            flicker1.setPosition(flickerRestPos1);
            flicker2.setPosition(flickerRestPos2);
            shootState = ShootState.IDLE;
            return;
        }

        switch (shootState) {
            case IDLE:
                if (shootRequested && ballCount > 0) {
                    // Build shoot order from loaded slots
                    buildShootOrder();
                    if (shootOrderLen == 0) {
                        shootState = ShootState.IDLE;
                        return;
                    }
                    shootSlotIndex = 0;
                    // Move to first loaded slot
                    currentSlot = shootOrder[0];
                    spindexer.setPosition(SLOT_POSITIONS[currentSlot]);
                    // Start flywheel
                    flywheel.setVelocity(rpmToTicksPerSec(shootRpm));
                    shootTimer.reset();
                    shootState = ShootState.SPINNING_UP;
                }
                break;

            case SPINNING_UP:
                // Wait for flywheel to reach speed
                if (shootTimer.seconds() >= SPINUP_TIME_SEC) {
                    // Fire!
                    flicker1.setPosition(flickerShootPos1);
                    flicker2.setPosition(flickerShootPos2);
                    shootTimer.reset();
                    shootState = ShootState.FIRING;
                }
                break;

            case FIRING:
                if (shootTimer.seconds() >= FLICK_TIME_SEC) {
                    // Retract flickers
                    flicker1.setPosition(flickerRestPos1);
                    flicker2.setPosition(flickerRestPos2);

                    // Mark slot as empty
                    slotLoaded[currentSlot] = false;
                    slotColor[currentSlot] = "NONE";
                    ballCount--;

                    shootSlotIndex++;
                    if (shootSlotIndex >= shootOrderLen) {
                        // All shots fired
                        flywheel.setVelocity(0);
                        shootState = ShootState.DONE;
                        shootTimer.reset();
                        return;
                    }

                    // Advance to next loaded slot
                    currentSlot = shootOrder[shootSlotIndex];
                    spindexer.setPosition(SLOT_POSITIONS[currentSlot]);
                    shootTimer.reset();
                    shootState = ShootState.ADVANCING;
                }
                break;

            case ADVANCING:
                // Wait for spindexer to arrive at next slot
                if (shootTimer.seconds() >= ADVANCE_TIME_SEC) {
                    // Fire next
                    flicker1.setPosition(flickerShootPos1);
                    flicker2.setPosition(flickerShootPos2);
                    shootTimer.reset();
                    shootState = ShootState.FIRING;
                }
                break;

            case DONE:
                // Brief cooldown, then return to idle
                if (shootTimer.seconds() >= 0.2) {
                    shootState = ShootState.IDLE;
                }
                break;
        }
    }

    /**
     * Build shoot order: loaded slots from one end to the other.
     * If p1 is loaded, go p1→p2→p3. If only p3 is loaded first, go p3→p2→p1.
     * This minimizes travel distance.
     */
    private void buildShootOrder() {
        shootOrder = new int[3];
        shootOrderLen = 0;

        // Determine direction: start from whichever end has a ball
        if (slotLoaded[0]) {
            // Forward: p1 → p2 → p3
            for (int i = 0; i < 3; i++) {
                if (slotLoaded[i]) {
                    shootOrder[shootOrderLen++] = i;
                }
            }
        } else {
            // Reverse: p3 → p2 → p1
            for (int i = 2; i >= 0; i--) {
                if (slotLoaded[i]) {
                    shootOrder[shootOrderLen++] = i;
                }
            }
        }
    }

    // ==========================================================================
    //  UTILITY
    // ==========================================================================

    /**
     * Reset all state — call on init or when manually resetting
     */
    public void reset() {
        slotLoaded = new boolean[]{false, false, false};
        slotColor  = new String[]{"NONE", "NONE", "NONE"};
        currentSlot = 0;
        ballCount = 0;
        intakeState = IntakeState.IDLE;
        shootState  = ShootState.IDLE;
        intakeOrderIndex = 0;
        intakeActive = false;
    }

    /**
     * Manually mark a slot as loaded (e.g., for preloads)
     */
    public void preload(int slot, String color) {
        if (slot >= 0 && slot < 3) {
            slotLoaded[slot] = true;
            slotColor[slot] = color;
            ballCount = 0;
            for (boolean b : slotLoaded) if (b) ballCount++;
        }
    }

    /**
     * Convert RPM to ticks/sec for GoBilda motor velocity control
     * GoBilda 5202 series: 28 ticks/rev
     */
    private double rpmToTicksPerSec(double rpm) {
        return rpm * 28.0 / 60.0;
    }

    // ========== Getters for telemetry / external logic ==========

    public boolean isIntakeActive()   { return intakeActive; }
    public boolean isFull()           { return ballCount >= 3; }
    public boolean isEmpty()          { return ballCount == 0; }
    public int     getBallCount()     { return ballCount; }
    public int     getCurrentSlot()   { return currentSlot; }
    public boolean isSlotLoaded(int i){ return (i >= 0 && i < 3) && slotLoaded[i]; }
    public String  getSlotColor(int i){ return (i >= 0 && i < 3) ? slotColor[i] : "NONE"; }
    public IntakeState getIntakeState() { return intakeState; }
    public ShootState  getShootState()  { return shootState; }
    public boolean isShooting()       { return shootState != ShootState.IDLE && shootState != ShootState.DONE; }

    /**
     * Add spindexer telemetry to the opmode
     */
    public void addTelemetry(org.firstinspires.ftc.robotcore.external.Telemetry telemetry) {
        telemetry.addData("", "--- Spindexer ---");
        telemetry.addData("Balls", ballCount + "/3");
        telemetry.addData("Slots",
                (slotLoaded[0] ? "●" : "○") + " " +
                (slotLoaded[1] ? "●" : "○") + " " +
                (slotLoaded[2] ? "●" : "○"));
        telemetry.addData("Colors",
                slotColor[0] + " | " + slotColor[1] + " | " + slotColor[2]);
        telemetry.addData("Active Slot", "p" + (currentSlot + 1));
        telemetry.addData("Intake", intakeState.toString());
        telemetry.addData("Shoot", shootState.toString());
    }
}
