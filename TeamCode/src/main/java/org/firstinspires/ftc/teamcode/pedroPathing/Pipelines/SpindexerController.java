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

    // ========== Slot positions (degrees — RTPAxon) ==========
    // 3 slots spaced 120° apart, two base angles for intake vs shoot
    private static final double SLOT_SPACING = 120.0;
    private static final double INTAKE_BASE = 67.0;
    private static final double SHOOT_BASE  = 36.3;

    // Derived positions
    public static final double INTAKE_P1 = wrap(INTAKE_BASE);
    public static final double INTAKE_P2 = wrap(INTAKE_BASE + SLOT_SPACING);
    public static final double INTAKE_P3 = wrap(INTAKE_BASE + SLOT_SPACING * 2);
    public static final double SHOOT_P1  = wrap(SHOOT_BASE);
    public static final double SHOOT_P2  = wrap(SHOOT_BASE + SLOT_SPACING);
    public static final double SHOOT_P3  = wrap(SHOOT_BASE + SLOT_SPACING * 2);

    private static final double[] INTAKE_POSITIONS = {INTAKE_P1, INTAKE_P2, INTAKE_P3};
    private static final double[] SHOOT_POSITIONS  = {SHOOT_P1,  SHOOT_P2,  SHOOT_P3};

    /** Wrap degrees to 0–360 */
    private static double wrap(double deg) {
        deg %= 360;
        if (deg < 0) deg += 360;
        return deg;
    }

    // Legacy aliases for external code that references P1/P2/P3
    public static final double P1 = INTAKE_P1;
    public static final double P2 = INTAKE_P2;
    public static final double P3 = INTAKE_P3;

    // ========== Timing constants ==========
    private static final double SETTLE_SEC    = 0.5;   // wait after servo arrives before reading sensor
    private static final double CONFIRM_SEC   = 0.1;   // ball must be seen continuously this long
    private static final double HOLD_SEC      = 0.05;  // pause after ball confirmed before rotating

    private static final double SPINUP_SEC    = 1.5;   // flywheel spin-up time
    public static double FLICK_SEC     = 0.25;   // flicker hold time — how long flicker stays extended
    public static double POST_FLICK_SEC = 0.25;    // pause after flicker retracts before advancing (must be long enough for servo to fully return)
    public static double SHOOT_SETTLE_SEC = 0.25; // settle time before firing each shot

    // ========== Misfire retry ==========
    private static final int    MAX_RETRIES       = 1;    // how many times to retry a misfire before giving up
    private static final double RETRY_EXTRA_RPM   = 200;  // extra RPM added on retry attempt
    private int retryCount = 0;                           // current retry count for this slot
    private int totalMisfires = 0;                        // lifetime misfire counter for telemetry

    // ========== Servo movement speed ==========
    private double servoTarget = INTAKE_P1; // where we're heading (degrees)
    private ElapsedTime servoTimer = new ElapsedTime();  // for time-based spindexer stepping (legacy)

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
    private int shootDirection = -1;  // -1 = P3→P1, +1 = P1→P3 (set dynamically)

    // Flicker positions
    private double flickRest  = 0;
    private double flickShoot = 0.375;

    // Flywheel
    private double shootRpm = 3000;
    private double activeShootRpm = 0;  // the RPM actually commanded during shooting (incl. boost)

    // ========== RPM drop detection ==========
    private static final double RPM_DROP_THRESHOLD = 150;  // RPM drop that indicates a ball was fired
    public static double FIRST_SHOT_BOOST  = 100;   // extra RPM added ONLY to the 1st shot
    private static final double RPM_BOOST_2ND_3RD = 0;   // extra RPM for 2nd and 3rd shots
    private double preFlickTPS = 0;   // flywheel velocity captured right before flick
    private boolean shotDetected = false;  // true if RPM drop seen during this flick
    private boolean[] slotEmpty = {true, true, true};  // tracks which slots are actually empty
    private int shotNumber = 0;  // 0 = first shot, 1 = second, 2 = third

    // ========== Shot counting (confirmed via RPM drop) ==========
    private int confirmedShots = 0;   // how many shots had a confirmed RPM drop this sequence
    private int totalShotsFired = 0;  // lifetime confirmed shots across all sequences

    // ========== Motif ordering ==========
    private String[] motifOrder = null;   // null = default P3→P2→P1, else color sequence
    private int motifIndex = 0;           // which motif position we're shooting next (0, 1, 2)

    // ========== Constructor ==========
    public SpindexerController() {}

    public void setFlickerPositions(double rest, double shoot) {
        flickRest = rest; flickShoot = shoot;
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
        servoTarget = INTAKE_POSITIONS[startSlot];
    }

    /** Get start slot index based on direction */
    private int startSlot() { return startFromP3 ? 2 : 0; }
    /** Get end slot index based on direction */
    private int endSlot()   { return startFromP3 ? 0 : 2; }
    /** Check if we're at the last slot in the fill direction */
    private boolean isLastSlot() { return currentSlot == endSlot(); }
    /** Get next EMPTY slot in fill direction, or -1 if none available */
    private int nextSlot() {
        int next = currentSlot + direction;
        // Skip over already-filled slots
        while (next >= 0 && next <= 2 && !slotEmpty[next]) {
            next += direction;
        }
        return (next >= 0 && next <= 2) ? next : -1;
    }

    /** Find ANY empty slot in the spindexer, closest to current position first. Returns -1 if all full. */
    private int findNearestEmptySlot() {
        int best = -1;
        double bestDist = Double.MAX_VALUE;
        double currentAngle = spindexerAxon.getRawAngle();
        for (int i = 0; i < 3; i++) {
            if (slotEmpty[i]) {
                double dist = Math.abs(shortestAngleDist(currentAngle, INTAKE_POSITIONS[i]));
                if (dist < bestDist) {
                    bestDist = dist;
                    best = i;
                }
            }
        }
        return best;
    }

    /** Shortest signed angle distance from a to b (wraps around 360°) */
    private double shortestAngleDist(double from, double to) {
        double d = to - from;
        while (d > 180) d -= 360;
        while (d < -180) d += 360;
        return d;
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
        // Update RTPAxon PID every loop
        spindexerAxon.update();

        switch (iState) {

            case IDLE:
                // Start sensing: finish any pending move, or settle at current slot
                if (!spindexerAxon.isAtTarget(5)) {
                    iState = IState.MOVING;
                } else {
                    iState = IState.SETTLING;
                    iTimer.reset();
                }
                break;

            case SETTLING:
                // If this slot already has a ball saved, skip to next empty slot
                if (!slotEmpty[currentSlot]) {
                    int nearest = findNearestEmptySlot();
                    if (nearest >= 0) {
                        currentSlot = nearest;
                        servoTarget = INTAKE_POSITIONS[currentSlot];
                        spindexerAxon.setTargetRotation(servoTarget);
                        iState = IState.MOVING;
                    } else {
                        // All slots full — go idle (top-of-loop guard will also catch this)
                        iState = IState.IDLE;
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
                // Ball confirmed — wait HOLD_SEC then move to next empty slot
                if (iTimer.seconds() >= HOLD_SEC) {
                    int nearest = findNearestEmptySlot();
                    if (nearest >= 0) {
                        // Move to closest empty slot
                        currentSlot = nearest;
                        servoTarget = INTAKE_POSITIONS[currentSlot];
                        spindexerAxon.setTargetRotation(servoTarget);
                        iState = IState.MOVING;
                    } else {
                        // All slots full — go idle
                        iState = IState.IDLE;
                    }
                }
                break;

            case MOVING:
                // RTPAxon handles the motion via PID — just check if arrived
                if (spindexerAxon.isAtTarget(5)) {
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
            flicker.setPosition(flickRest);
            sState = SState.IDLE;
            return;
        }

        // Update RTPAxon PID every loop — during FIRE/RETRACTING, actively
        // brake the CRServo (power=0) instead of skipping update entirely,
        // so the servo doesn't float and the PID doesn't accumulate drift.
        if (sState == SState.FIRE || sState == SState.RETRACTING) {
            spindexerAxon.setPower(0);   // active brake — holds position
        } else {
            spindexerAxon.update();       // normal PID control
        }

        switch (sState) {

            case IDLE:
                if (shootPressed) {
                    // Always assume all 3 slots are loaded when we shoot
                    prefillAllSlots();
                    confirmedShots = 0;  // reset per-sequence counter

                    // Find first slot to shoot (nearest end)
                    shootSlot = findFirstShootSlot();

                    currentSlot = shootSlot;
                    servoTarget = SHOOT_POSITIONS[shootSlot];
                    spindexerAxon.setTargetRotation(servoTarget);
                    // Ensure flicker at rest
                    flicker.setPosition(flickRest);
                    shotNumber = 0;  // first shot
                    retryCount = 0;  // fresh retries for this slot
                    // First shot gets its own boost
                    activeShootRpm = shootRpm + FIRST_SHOT_BOOST;
                    flywheel.setVelocity(rpmToTPS(activeShootRpm));
                    sTimer.reset();
                    sState = SState.SPINUP;
                }
                break;

            case SPINUP:
                // Continuously re-apply boosted velocity so it doesn't get overridden
                flywheel.setVelocity(rpmToTPS(activeShootRpm));
                // RTPAxon handles servo motion — check if arrived
                boolean servoReady = spindexerAxon.isAtTarget(5);
                // Wait for BOTH flywheel speed AND servo arrival
                if (sTimer.seconds() >= SPINUP_SEC && servoReady) {
                    sTimer.reset();
                    sState = SState.SHOOT_SETTLE;
                }
                break;

            case SHOOT_SETTLE:
                // Keep flywheel at target while settling
                flywheel.setVelocity(rpmToTPS(activeShootRpm));
                // Wait for spindexer to settle, then fire unconditionally
                if (sTimer.seconds() >= SHOOT_SETTLE_SEC) {
                    preFlickTPS = flywheel.getVelocity();
                    shotDetected = false;
                    flicker.setPosition(flickShoot);
                    sTimer.reset();
                    sState = SState.FIRE;
                }
                break;

            case FIRE:
                // Check for RPM drop — counts confirmed shots
                if (!shotDetected) {
                    double currentTPS = flywheel.getVelocity();
                    if (preFlickTPS - currentTPS > rpmToTPS(RPM_DROP_THRESHOLD)) {
                        shotDetected = true;
                        confirmedShots++;
                        totalShotsFired++;
                    }
                }
                // Wait for flick to complete
                if (sTimer.seconds() >= FLICK_SEC) {
                    // Command flicker back to rest
                    flicker.setPosition(flickRest);

                    // Mark slot as fired
                    slotColor[shootSlot] = "NONE";
                    slotEmpty[shootSlot] = true;

                    // Advance to next slot in the shoot queue
                    shootQueueIndex++;
                    if (shootQueueIndex >= 3) {
                        // All 3 slots fired — done
                        flywheel.setVelocity(0);
                        sTimer.reset();
                        sState = SState.DONE;
                    } else {
                        // Advance to next slot in queue
                        shootSlot = shootQueue[shootQueueIndex];
                        shotNumber++;
                        // Set RPM for 2nd and 3rd shots
                        activeShootRpm = shootRpm;
                        flywheel.setVelocity(rpmToTPS(activeShootRpm));
                        sTimer.reset();
                        sState = SState.RETRACTING;
                    }
                }
                break;

            case RETRACTING:
                // Wait for flicker to retract before moving spindexer
                if (sTimer.seconds() >= POST_FLICK_SEC) {
                    currentSlot = shootSlot;
                    servoTarget = SHOOT_POSITIONS[shootSlot];
                    spindexerAxon.setTargetRotation(servoTarget);
                    sState = SState.ADVANCING;
                }
                break;

            case ADVANCING:
                // RTPAxon handles the motion — just check if arrived
                if (spindexerAxon.isAtTarget(5)) {
                    // Arrived — settle before firing
                    sTimer.reset();
                    sState = SState.SHOOT_SETTLE;
                }
                break;

            case DONE:
                if (sTimer.seconds() >= 0.3) {
                    // Move spindexer back to first intake position so intake can resume
                    currentSlot = 0;
                    servoTarget = INTAKE_POSITIONS[0];
                    spindexerAxon.setTargetRotation(servoTarget);
                    // Clear all slots — everything was shot
                    for (int i = 0; i < 3; i++) {
                        slotEmpty[i] = true;
                        slotColor[i] = "NONE";
                    }
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
     * Picks the nearest slot (P1, P2, or P3), builds a shoot queue from there.
     * From P2: fires P2 → nearest neighbor → far side (e.g. P2→P1→P3 or P2→P3→P1)
     * From an end: sweeps linearly (P1→P2→P3 or P3→P2→P1)
     */
    private int[] shootQueue = new int[3];  // ordered slot indices to fire
    private int shootQueueIndex = 0;        // which queue entry we're on

    private int findFirstShootSlot() {
        double currentAngle = spindexerAxon.getRawAngle();
        double distToP1 = Math.abs(shortestAngleDist(currentAngle, SHOOT_P1));
        double distToP2 = Math.abs(shortestAngleDist(currentAngle, SHOOT_P2));
        double distToP3 = Math.abs(shortestAngleDist(currentAngle, SHOOT_P3));

        if (distToP2 <= distToP1 && distToP2 <= distToP3) {
            // Closest to P2 — shoot P2 first, then nearest neighbor, then far side
            shootQueue[0] = 1;
            if (distToP1 <= distToP3) {
                shootQueue[1] = 0; shootQueue[2] = 2;  // P2→P1→P3
            } else {
                shootQueue[1] = 2; shootQueue[2] = 0;  // P2→P3→P1
            }
        } else if (distToP1 <= distToP3) {
            // Closest to P1 — sweep P1→P2→P3
            shootQueue[0] = 0; shootQueue[1] = 1; shootQueue[2] = 2;
        } else {
            // Closest to P3 — sweep P3→P2→P1
            shootQueue[0] = 2; shootQueue[1] = 1; shootQueue[2] = 0;
        }

        shootQueueIndex = 0;
        shootDirection = 1;  // legacy field, not used for ordering anymore
        return shootQueue[0];
    }

    /**
     * Find the next slot to shoot (simple sequential — always fires all 3).
     */
    private int findNextShootSlot() {
        int next = shootSlot + shootDirection;
        return (next >= 0 && next <= 2) ? next : -1;
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
     * Set flicker directly to target position.
     */
    private void setFlicker(double pos) {
        flicker.setPosition(pos);
    }

    /**
     * (Legacy — no longer used. RTPAxon PID handles spindexer motion now.)
     */
    private boolean stepServo(double stepPerSec) {
        return spindexerAxon.isAtTarget(5);
    }

    /** Move spindexer to a specific slot (for init / manual use) */
    public void goToSlot(int slot) {
        if (slot >= 0 && slot <= 2) {
            currentSlot = slot;
            servoTarget = INTAKE_POSITIONS[slot];
            spindexerAxon.setTargetRotation(servoTarget);
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
        retryCount = 0;
        flicker.setPosition(flickRest);
    }

    // ========== Getters ==========
    public int     getCurrentSlot()   { return currentSlot; }
    public String  getSlotColor(int i){ return (i >= 0 && i < 3) ? slotColor[i] : "NONE"; }
    public boolean isSlotEmpty(int i) { return (i >= 0 && i < 3) && slotEmpty[i]; }
    public boolean isShooting()       { return sState != SState.IDLE && sState != SState.DONE; }
    public int     getBallCount()     { int c = 0; for (boolean e : slotEmpty) if (!e) c++; return c; }
    /** Number of open slots available for intake (0-3) */
    public int     getOpenSlots()     { return 3 - getBallCount(); }
    /** Total misfire count since last reset (for diagnostics) */
    public int     getTotalMisfires() { return totalMisfires; }
    /** Reset misfire counter (call at start of match) */
    public void    resetMisfireCount(){ totalMisfires = 0; }
    /** Confirmed shots this sequence (RPM drop detected). Resets when shoot starts. */
    public int     getConfirmedShots(){ return confirmedShots; }
    /** How many shots were NOT confirmed this sequence (3 - confirmedShots) */
    public int     getMissedShots()   { return 3 - confirmedShots; }
    /** Lifetime total confirmed shots across all sequences */
    public int     getTotalShotsFired(){ return totalShotsFired; }
    /** Reset lifetime counter */
    public void    resetTotalShotsFired(){ totalShotsFired = 0; }

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
        telemetry.addData("Open Slots", getOpenSlots());
        telemetry.addData("Confirmed Shots", confirmedShots + "/3");
        if (sState != SState.IDLE && sState != SState.DONE) {
            telemetry.addData("Flywheel Target", String.format("%.0f RPM (boost: +%.0f)", activeShootRpm, activeShootRpm - shootRpm));
        }
        if (sState == SState.FIRE) {
            telemetry.addData("Shot Detected", shotDetected ? "YES (RPM drop)" : "waiting...");
            telemetry.addData("Pre-Flick RPM", String.format("%.0f", preFlickTPS * 60.0 / 28.0));
        } else if (sState == SState.RETRACTING || sState == SState.ADVANCING) {
            telemetry.addData("Last Shot", shotDetected ? "FIRED" : "MISSED");
        }
        telemetry.addData("Total Shots Fired", totalShotsFired);
        telemetry.addData("Spindexer Angle", String.format("%.1f°", spindexerAxon.getRawAngle()));
        telemetry.addData("Spindexer Target", String.format("%.1f°", servoTarget));
        telemetry.addData("Ball Present?", isBallPresent() ? "YES" : "NO");
        telemetry.addData("Verified Color", detectBallColor());
        telemetry.addData("Sensor Compliance?", sensorsAgree() ? "YES" : "NO");
    }
}
