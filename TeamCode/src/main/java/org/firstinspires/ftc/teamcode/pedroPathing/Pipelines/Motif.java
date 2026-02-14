package org.firstinspires.ftc.teamcode.pedroPathing.Pipelines;

/**
 * Motif — maps obelisk AprilTag IDs to the required ball shoot order.
 *
 * Each obelisk has a unique tag that tells us which color goes in which
 * scoring position (slot 0 = first scored, slot 2 = last scored):
 *
 *   Tag 21 → GREEN, PURPLE, PURPLE
 *   Tag 22 → PURPLE, GREEN, PURPLE
 *   Tag 23 → PURPLE, PURPLE, GREEN
 *
 * Usage:
 *   int tagId = Limelight_Pipeline.getObeliskTagId();   // during init_loop
 *   String[] order = Motif.getShootOrder(tagId);         // {"GREEN","PURPLE","PURPLE"}
 *   spindexer.setMotifOrder(order);                      // before shooting
 */
public class Motif {

    // Shoot-order arrays: index 0 = first ball to fire, index 2 = last
    private static final String[] ORDER_21 = {"GREEN",  "PURPLE", "PURPLE"};
    private static final String[] ORDER_22 = {"PURPLE", "GREEN",  "PURPLE"};
    private static final String[] ORDER_23 = {"PURPLE", "PURPLE", "GREEN"};

    // Default order when no obelisk detected (just fire whatever is loaded)
    private static final String[] ORDER_DEFAULT = {"ANY", "ANY", "ANY"};

    /**
     * Get the required shoot order for a given obelisk tag ID.
     * @param tagId  21, 22, or 23  (-1 or anything else → default)
     * @return String[3]: first-to-fire … last-to-fire color requirement
     */
    public static String[] getShootOrder(int tagId) {
        switch (tagId) {
            case 21: return ORDER_21.clone();
            case 22: return ORDER_22.clone();
            case 23: return ORDER_23.clone();
            default: return ORDER_DEFAULT.clone();
        }
    }

    /**
     * Human-readable motif name for telemetry.
     */
    public static String getMotifName(int tagId) {
        switch (tagId) {
            case 21: return "GPP (Tag 21)";
            case 22: return "PGP (Tag 22)";
            case 23: return "PPG (Tag 23)";
            default: return "UNKNOWN";
        }
    }

    /**
     * Check if a slot color satisfies a motif requirement.
     * "ANY" matches any non-empty slot.
     */
    public static boolean colorMatches(String required, String slotColor) {
        if ("ANY".equals(required)) return !"NONE".equals(slotColor);
        return required.equals(slotColor);
    }
}
