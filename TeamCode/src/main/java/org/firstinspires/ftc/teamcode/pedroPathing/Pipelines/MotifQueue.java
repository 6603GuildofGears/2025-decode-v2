package org.firstinspires.ftc.teamcode.pedroPathing.Pipelines;

/**
 * Motif Queue Builder — sorts spindexer slots to match the required color sequence.
 * 
 * Given:
 *   - motifOrder = ["GREEN", "PURPLE", "PURPLE"] (from obelisk tag)
 *   - slotColors = ["PURPLE", "GREEN", "PURPLE"] (from color scan)
 * 
 * Builds:
 *   - shootQueue = [1, 0, 2] (fire slot 1 first, then 0, then 2)
 * 
 * This ensures balls are shot in the correct color order for motif bonus points.
 */
public class MotifQueue {

    /**
     * Build a shoot queue that matches the motif order.
     * 
     * @param motifOrder  Required color sequence (e.g., ["GREEN", "PURPLE", "PURPLE"])
     * @param slotColors  Scanned colors in each slot (e.g., ["PURPLE", "GREEN", "PURPLE"])
     * @param slotEmpty   Which slots are empty (true = empty, false = has ball)
     * @return shootQueue Array of slot indices in firing order [0-2]
     * 
     * Algorithm:
     *   1. Loop through motif positions (1st, 2nd, 3rd required color)
     *   2. For each required color, find the first available slot with that color
     *   3. Mark that slot as "used" so it doesn't get picked again
     *   4. If no match found, pick the first remaining slot (wrong color = 0 motif points but still basket points)
     */
    public static int[] buildMotifQueue(String[] motifOrder, String[] slotColors, boolean[] slotEmpty) {
        int[] shootQueue = new int[3];
        boolean[] slotUsed = new boolean[3]; // Track which slots are already assigned
        
        // Loop through each position in the motif order
        for (int motifPos = 0; motifPos < 3; motifPos++) {
            String requiredColor = motifOrder[motifPos];
            int bestSlot = -1;
            
            // Search for a slot with the required color that hasn't been used yet
            for (int slot = 0; slot < 3; slot++) {
                if (!slotUsed[slot] && !slotEmpty[slot] && slotColors[slot].equalsIgnoreCase(requiredColor)) {
                    bestSlot = slot;
                    break; // Found a match
                }
            }
            
            // If no match found, pick the first remaining non-empty slot (wrong color, but still score points)
            if (bestSlot == -1) {
                for (int slot = 0; slot < 3; slot++) {
                    if (!slotUsed[slot] && !slotEmpty[slot]) {
                        bestSlot = slot;
                        break;
                    }
                }
            }
            
            // If still no slot (all empty or all used), just pick the first unused slot
            if (bestSlot == -1) {
                for (int slot = 0; slot < 3; slot++) {
                    if (!slotUsed[slot]) {
                        bestSlot = slot;
                        break;
                    }
                }
            }
            
            shootQueue[motifPos] = bestSlot;
            slotUsed[bestSlot] = true; // Mark this slot as assigned
        }
        
        return shootQueue;
    }
    
    /**
     * Helper: Check if the queue matches the motif perfectly.
     * Returns how many positions match (0-3).
     */
    public static int countMotifMatches(String[] motifOrder, String[] slotColors, int[] shootQueue) {
        int matches = 0;
        for (int i = 0; i < 3; i++) {
            int slotToFire = shootQueue[i];
            String colorToFire = slotColors[slotToFire];
            String requiredColor = motifOrder[i];
            if (colorToFire.equalsIgnoreCase(requiredColor)) {
                matches++;
            }
        }
        return matches;
    }
    
    /**
     * Debug string showing the shoot order.
     * Example: "Queue: [1,0,2] → G,P,P (3/3 match)"
     */
    public static String debugString(String[] motifOrder, String[] slotColors, int[] shootQueue, boolean[] slotEmpty) {
        StringBuilder sb = new StringBuilder("Queue: [");
        for (int i = 0; i < 3; i++) {
            sb.append(shootQueue[i]);
            if (i < 2) sb.append(",");
        }
        sb.append("] → ");
        
        for (int i = 0; i < 3; i++) {
            int slot = shootQueue[i];
            if (slotEmpty[slot]) {
                sb.append("EMPTY");
            } else {
                String color = slotColors[slot];
                sb.append(color.substring(0, 1)); // G or P
            }
            if (i < 2) sb.append(",");
        }
        
        int matches = countMotifMatches(motifOrder, slotColors, shootQueue);
        sb.append(" (").append(matches).append("/3 match)");
        
        return sb.toString();
    }
}
