package org.firstinspires.ftc.teamcode.pedroPathing.Pipelines;

import com.qualcomm.robotcore.util.ElapsedTime;
import com.bylazar.configurables.annotations.Configurable;

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
 *   - Fires all 3 slots: P3 → P2 → P1 (reverse of intake order).
 *   - Spins up flywheel → flick → advance → flick → advance → flick → done.
 */
@Configurable
public class SpindexerController {

    // ========== Slot positions (servo
    //  values) ==========
    public static final double P1 = 0.02;
    public static final double P2 = 0.375;
    public static final double P3 = 0.75;
    private static final double[] POSITIONS = {P1, P2, P3};

    // ========== Timing constants ==========
    private static final double SETTLE_SEC    = 0.35;  // wait after servo arrives before reading sensor
    private static final double CONFIRM_SEC   = 0.03;  // ball must be seen continuously this long
    private static final double HOLD_SEC      = 0.05;  // pause after ball confirmed before rotating

    private static final double SPINUP_SEC    = 1;   // flywheel spin-up time
    public static double FLICK_SEC     = 0.45;   // flicker hold time — how long flicker stays extended
    public static double FLICK2_DELAY  = 0.02;   // right flicker fires after left
    public static double POST_FLICK_SEC = 0.5;   // pause after flicker retracts before advancing
    public static double SHOOT_SETTLE_SEC = 0.5; // settle time before firing each shot

    // ========== Servo movement speed ==========
    private static final double INTAKE_SERVO_STEP = 0.03;  // intake rotation speed
    private static final double SHOOT_SERVO_STEP  = 0.0175;  // shoot rotation speed
    public static double FLICKER_STEP = 0.04;   // flicker speed per loop (smaller = slower flick)
    private double servoPos  = P1;   // current commanded position
    private double servoTarget = P1; // where we're heading

    // Flicker position tracking (for smooth stepping)
    private double flick1Pos = 0.06875;
    private double flick2Pos = 0.05;
    private double flick1Target = 0.06875;
    private double flick2Target = 0.05;

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
    //  IDLE → SPINUP → SHOOT_SETTLE → FIRE → RETRACTING → ADVANCING → SHOOT_SETTLE → FIRE → ... → DONE → IDLE
    private enum SState { IDLE, SPINUP, SHOOT_SETTLE, FIRE, RETRACTING, ADVANCING, DONE }
    private SState sState = SState.IDLE;
    private ElapsedTime sTimer = new ElapsedTime();
    private int shootSlot = 0;  // which slot we're firing (0, 1, 2)

    // Flicker positions
    private double flickRest1  = 0.06875;
    private double flickRest2  = 0.05;
    private double flickShoot1 = 0.53;
    private double flickShoot2 = 0.5;

    // Flywheel
    private double shootRpm = 3000;

    // ========== RPM drop detection ==========
    private static final double RPM_DROP_THRESHOLD = 150;  // RPM drop that indicates a ball was fired
    private static final double FIRST_SHOT_REDUCE = 200;   // lower 1st shot RPM by this much
    private static final double RPM_BOOST_2ND_3RD = 150;   // extra RPM for 2nd and 3rd shots
    private double preFlickTPS = 0;   // flywheel velocity captured right before flick
    private boolean shotDetected = false;  // true if RPM drop seen during this flick
    private boolean[] slotEmpty = {true, true, true};  // tracks which slots are actually empty
    private int shotNumber = 0;  // 0 = first shot, 1 = second, 2 = third

    // ========== Motif ordering ==========
    private String[] motifOrder = null;   // null = default P3→P2→P1, else color sequence
    private int motifIndex = 0;           // which motif position we're shooting next (0, 1, 2)

    // ========== Constructor ==========
    public SpindexerController() {}

    public void setFlickerPositions(double r1, double r2, double s1, double s2) {
        flickRest1 = r1; flickRest2 = r2; flickShoot1 = s1; flickShoot2 = s2;
        // Init tracking to rest positions
        flick1Pos = r1; flick1Target = r1;
        flick2Pos = r2; flick2Target = r2;
    }

    public void setShootRpm(double rpm) { this.shootRpm = rpm; }

    /**
     * Set motif color order for shooting.
     * Pass null to revert to default P3→P2→P1 order.
     * @param order  String[3] e.g. {"GREEN","PURPLE","PURPLE"} — first-to-fire first
     */
    public void setMotifOrder(String[] order) {
        this.motifOrder = order;
        this.motifIndex = 0;
    }

    /** Clear motif ordering — revert to default shoot order */
    public void clearMotifOrder() {
        this.motifOrder = null;
        this.motifIndex = 0;
    }

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

        // Refresh color sensor data each loop
        updateSensors();

        // --- Don't process spindexer rotation while shooting ---
        if (isShooting()) {
            iState = IState.IDLE;
            ballSeen = false;
            return false;
        }

        // --- Auto-spin state machine runs ALWAYS (trigger only controls motor) ---
        switch (iState) {

            case IDLE:
                // Start sensing: finish any pending move, or settle at current slot
                if (Math.abs(servoPos - servoTarget) > 0.01) {
                    iState = IState.MOVING;
                } else {
                    iState = IState.SETTLING;
                    iTimer.reset();
                }
                break;

            case SETTLING:
                // If this slot already has a ball saved, skip to next slot
                if (!slotEmpty[currentSlot]) {
                    int next = nextSlot();
                    if (next >= 0) {
                        currentSlot = next;
                        servoTarget = POSITIONS[currentSlot];
                        iState = IState.MOVING;
                    } else {
                        // All slots full — stay idle, nothing to do
                    }
                    break;
                }
                // Wait for servo to physically stop before trusting sensor
                if (iTimer.seconds() >= SETTLE_SEC) {
                    iState = IState.SENSING;
                    ballSeen = false;
                }
                break;

            case SENSING:
                // Read sensor — look for a ball
                if (isBallPresent()) {
                    // Save color IMMEDIATELY on first detection so it persists
                    slotColor[currentSlot] = detectBallColor();
                    slotEmpty[currentSlot] = false;

                    if (!ballSeen) {
                        // First frame — start confirmation timer for advancing
                        ballSeen = true;
                        iTimer.reset();
                    } else if (iTimer.seconds() >= CONFIRM_SEC) {
                        // Confirmed long enough — advance to next slot
                        iState = IState.DETECTED;
                        iTimer.reset();
                        ballSeen = false;
                    }
                } else {
                    // Lost it — restart confirmation (saved color stays)
                    ballSeen = false;
                }
                break;

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
                        // Already at last slot — stay here
                        iState = IState.SENSING;
                    }
                }
                break;

            case MOVING:
                // Slowly step servo toward target position
                if (stepServo(INTAKE_SERVO_STEP)) {
                    // Arrived — now settle before reading sensor
                    iState = IState.SETTLING;
                    iTimer.reset();
                }
                break;
        }

        // Trigger only controls whether intake motor runs
        return held;
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
            flick1Target = flickRest1;
            flick2Target = flickRest2;
            flick1Pos = flickRest1; flick2Pos = flickRest2;
            flicker1.setPosition(flickRest1);
            flicker2.setPosition(flickRest2);
            sState = SState.IDLE;
            return;
        }

        // Step flickers toward their targets every loop
        stepFlickers();

        switch (sState) {

            case IDLE:
                if (shootPressed) {
                    // Find first slot to shoot (always P3)
                    shootSlot = findFirstShootSlot();

                    currentSlot = shootSlot;
                    servoTarget = POSITIONS[shootSlot];
                    // Ensure flickers at rest
                    flick1Target = flickRest1;
                    flick2Target = flickRest2;
                    shotNumber = 0;  // first shot
                    flywheel.setVelocity(rpmToTPS(shootRpm - FIRST_SHOT_REDUCE));
                    sTimer.reset();
                    sState = SState.SPINUP;
                }
                break;

            case SPINUP:
                // Move servo toward target during spinup (hides travel behind spin-up time)
                boolean servoReady = stepServo(SHOOT_SERVO_STEP);
                // Wait for BOTH flywheel speed AND servo arrival
                if (sTimer.seconds() >= SPINUP_SEC && servoReady) {
                    sTimer.reset();
                    sState = SState.SHOOT_SETTLE;
                }
                break;

            case SHOOT_SETTLE:
                // Wait for spindexer to settle before firing
                if (sTimer.seconds() >= SHOOT_SETTLE_SEC) {
                    preFlickTPS = flywheel.getVelocity();  // capture baseline RPM
                    shotDetected = false;
                    // Command flickers to shoot position (stepped smoothly)
                    flick1Target = flickShoot1;
                    // flicker2 fires after FLICK2_DELAY in FIRE state
                    sTimer.reset();
                    sState = SState.FIRE;
                }
                break;

            case FIRE:
                // Fire right flicker after delay
                if (sTimer.seconds() >= FLICK2_DELAY) {
                    flick2Target = flickShoot2;
                }
                // Check for RPM drop — indicates ball was actually shot
                if (!shotDetected) {
                    double currentTPS = flywheel.getVelocity();
                    if (preFlickTPS - currentTPS > rpmToTPS(RPM_DROP_THRESHOLD)) {
                        shotDetected = true;
                    }
                }
                // Wait for flick to complete AND flickers to reach shoot position
                boolean flickersAtShoot = flickersArrived();
                if (sTimer.seconds() >= FLICK_SEC && flickersAtShoot) {
                    // Command flickers back to rest (stepped smoothly)
                    flick1Target = flickRest1;
                    flick2Target = flickRest2;

                    // Only mark slot empty if RPM drop confirmed the ball actually left.
                    // If no drop detected, the ball is likely still in the slot (misfire/jam).
                    if (shotDetected) {
                        slotColor[shootSlot] = "NONE";
                        slotEmpty[shootSlot] = true;
                    }
                    // shotDetected is also used for telemetry: true = fired, false = misfire

                    // Find next slot to shoot
                    int nextShoot = findNextShootSlot();
                    if (nextShoot < 0) {
                        // No more balls — done
                        flywheel.setVelocity(0);
                        sTimer.reset();
                        sState = SState.DONE;
                    } else {
                        // Pause before advancing to next filled slot
                        shootSlot = nextShoot;
                        shotNumber++;  // increment shot counter
                        // Boost RPM for 2nd and 3rd shots
                        double boostedRpm = shootRpm + (shotNumber >= 1 ? RPM_BOOST_2ND_3RD : 0);
                        flywheel.setVelocity(rpmToTPS(boostedRpm));
                        sTimer.reset();
                        sState = SState.RETRACTING;
                    }
                }
                break;

            case RETRACTING:
                // Wait for flickers to fully retract before moving spindexer
                if (sTimer.seconds() >= POST_FLICK_SEC && flickersArrived()) {
                    currentSlot = shootSlot;
                    servoTarget = POSITIONS[shootSlot];
                    sState = SState.ADVANCING;
                }
                break;

            case ADVANCING:
                // Slowly step spindexer to next slot, then settle before firing
                if (stepServo(SHOOT_SERVO_STEP)) {
                    // Arrived — settle before firing
                    sTimer.reset();
                    sState = SState.SHOOT_SETTLE;
                }
                break;

            case DONE:
                if (sTimer.seconds() >= 0.3) {
                    // Stay at current position — don't reset servo back to P1
                    sState = SState.IDLE;
                }
                break;
        }
    }

    // ======================================================================
    //  SHOOT SLOT SELECTION (motif-aware)
    // ======================================================================

    /**
     * Find the first slot to shoot.
     * Hard-coded: always starts at P3 (slot 2) regardless of slot status.
     */
    private int findFirstShootSlot() {
        return 2;  // always start at P3
    }

    /**
     * Find the next slot to shoot after the current one was fired.
     * Hard-coded: always goes P3→P2→P1 regardless of slot status.
     */
    private int findNextShootSlot() {
        int next = shootSlot - 1;
        return (next >= 0) ? next : -1;  // P2 then P1, then done
    }

    /**
     * Find a non-empty slot whose color matches motifOrder[idx].
     * Skips already-empty slots. Returns -1 if no match found.
     */
    private int findSlotForMotifIndex(int idx) {
        if (motifOrder == null || idx >= motifOrder.length) return -1;
        String needed = motifOrder[idx];
        // "ANY" matches any non-empty slot
        for (int i = 0; i < 3; i++) {
            if (!slotEmpty[i]) {
                if ("ANY".equals(needed) || needed.equals(slotColor[i])) {
                    return i;
                }
            }
        }
        // No color match — fall back to any non-empty slot (better than not shooting)
        for (int i = 0; i < 3; i++) {
            if (!slotEmpty[i]) return i;
        }
        return -1;
    }

    // ======================================================================
    //  UTILITY
    // ======================================================================

    private double rpmToTPS(double rpm) {
        return rpm * 28.0 / 60.0;  // GoBilda 28 ticks/rev
    }

    /**
     * Step both flicker servos toward their targets.
     * Called once per loop for smooth movement.
     */
    private void stepFlickers() {
        flick1Pos = stepToward(flick1Pos, flick1Target, FLICKER_STEP);
        flick2Pos = stepToward(flick2Pos, flick2Target, FLICKER_STEP);
        flicker1.setPosition(flick1Pos);
        flicker2.setPosition(flick2Pos);
    }

    /** Step a value toward a target by at most 'step' per call */
    private double stepToward(double current, double target, double step) {
        double diff = target - current;
        if (Math.abs(diff) <= step * 0.5) return target;
        return current + Math.signum(diff) * step;
    }

    /** Check if both flickers have reached their targets */
    private boolean flickersArrived() {
        return Math.abs(flick1Pos - flick1Target) < FLICKER_STEP * 0.5
            && Math.abs(flick2Pos - flick2Target) < FLICKER_STEP * 0.5;
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

    /**
     * Mark all 3 slots as loaded (for auto — robot starts pre-filled).
     * Color is set to "ANY" since auto doesn't care about color.
     */
    public void prefillAllSlots() {
        for (int i = 0; i < 3; i++) {
            slotEmpty[i] = false;
            slotColor[i] = "ANY";
        }
    }

    /** Mark all 3 slots as empty. */
    public void clearAllSlots() {
        for (int i = 0; i < 3; i++) {
            slotEmpty[i] = true;
            slotColor[i] = "NONE";
        }
    }

    /** Reset shoot state machine to IDLE so a new shoot sequence can start cleanly. */
    public void resetShootState() {
        sState = SState.IDLE;
        shotNumber = 0;
        // Sync flicker tracking to rest so stale positions don't cause an extra flick
        flick1Pos = flickRest1;
        flick2Pos = flickRest2;
        flick1Target = flickRest1;
        flick2Target = flickRest2;
    }

    // ========== Getters ==========
    public int     getCurrentSlot()   { return currentSlot; }
    public String  getSlotColor(int i){ return (i >= 0 && i < 3) ? slotColor[i] : "NONE"; }
    public boolean isSlotEmpty(int i) { return (i >= 0 && i < 3) && slotEmpty[i]; }
    public boolean isShooting()       { return sState != SState.IDLE && sState != SState.DONE; }
    public int     getBallCount()     { int c = 0; for (boolean e : slotEmpty) if (!e) c++; return c; }

    /** Add telemetry */
    public void addTelemetry(org.firstinspires.ftc.robotcore.external.Telemetry telemetry) {
        telemetry.addData("--- Spindexer ---", "");
        telemetry.addData("Current Slot", "P" + (currentSlot + 1));
        // Show slot colors — show LIVE reading when sensor sees a ball at current slot
        int liveBallCount = 0;
        for (int i = 0; i < 3; i++) {
            String label = "P" + (i + 1);
            boolean liveAtSlot = (i == currentSlot) && isBallPresent();
            if (liveAtSlot) {
                telemetry.addData(label, detectBallColor() + " (LIVE)");
                liveBallCount++;
            } else if (!slotEmpty[i]) {
                telemetry.addData(label, slotColor[i]);
                liveBallCount++;
            } else {
                telemetry.addData(label, "EMPTY");
            }
        }
        telemetry.addData("Balls Loaded", liveBallCount + "/3");
        telemetry.addData("Intake State", iState.toString());
        telemetry.addData("Shoot State", sState.toString());
        if (motifOrder != null) {
            telemetry.addData("Motif Order", motifOrder[0] + " → " + motifOrder[1] + " → " + motifOrder[2]);
            telemetry.addData("Motif Index", motifIndex);
        }
        if (sState == SState.FIRE) {
            telemetry.addData("Shot Detected", shotDetected ? "YES (RPM drop)" : "waiting...");
            telemetry.addData("Pre-Flick RPM", String.format("%.0f", preFlickTPS * 60.0 / 28.0));
        } else if (sState == SState.RETRACTING || sState == SState.ADVANCING) {
            telemetry.addData("Last Shot", shotDetected ? "FIRED" : "MISFIRE (ball kept)");
        }
        telemetry.addData("Servo Pos/Target", String.format("%.3f / %.3f", servoPos, servoTarget));
        telemetry.addData("Spindexer Pos", String.format("%.3f", spindexer.getPosition()));
        telemetry.addData("Ball Present?", isBallPresent() ? "YES" : "NO");
        telemetry.addData("Verified Color", detectBallColor());
        telemetry.addData("Sensor Compliance?", sensorsAgree() ? "YES" : "NO");
        telemetry.addData("Sensor1", getColorSensorDebug());
        telemetry.addData("Sensor2", getColorSensor2Debug());
    }
}
