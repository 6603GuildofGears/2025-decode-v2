package org.firstinspires.ftc.teamcode.pedroPathing.TeleOp;

import org.opencv.core.*;
import org.opencv.imgproc.Imgproc;
import org.opencv.imgproc.Moments;
import org.openftc.easyopencv.OpenCvPipeline;

import java.util.ArrayList;
import java.util.Collections;
import java.util.List;

/**
 * FTC Wiffle Ball Pattern Detector using EasyOpenCV
 * Detects purple and green wiffle balls on a red ramp.
 * 
 * For use with "da-ball-cam" webcam on FTC robot.
 * Converted from Python/OpenCV to Java/EasyOpenCV.
 */
public class WiffleBallPipeline extends OpenCvPipeline {
    
    // Ball colors enum
    public enum BallColor {
        PURPLE("P", "Purple"),
        GREEN("G", "Green"),
        UNKNOWN("?", "Unknown"),
        EMPTY("_", "Empty");
        
        private final String symbol;
        private final String name;
        BallColor(String symbol, String name) { 
            this.symbol = symbol; 
            this.name = name;
        }
        public String getSymbol() { return symbol; }
        public String getName() { return name; }
    }
    
    // Ball detection result
    public static class DetectedBall {
        public BallColor color;
        public Point center;
        public double area;
        public MatOfPoint contour;
        
        public DetectedBall(BallColor color, Point center, double area, MatOfPoint contour) {
            this.color = color;
            this.center = center;
            this.area = area;
            this.contour = contour;
        }
    }
    
    // HSV color ranges (same values as Python version)
    // Purple ball range
    private final Scalar purpleLower = new Scalar(113, 40, 30);
    private final Scalar purpleUpper = new Scalar(170, 255, 255);
    
    // Green ball range
    private final Scalar greenLower = new Scalar(78, 100, 50);
    private final Scalar greenUpper = new Scalar(100, 255, 255);
    
    // Silver bar detection
    private final Scalar silverLower = new Scalar(0, 0, 150);
    private final Scalar silverUpper = new Scalar(180, 50, 255);
    
    // Red ramp detection (red wraps around in HSV)
    private final Scalar redLower1 = new Scalar(0, 80, 80);
    private final Scalar redUpper1 = new Scalar(10, 255, 255);
    private final Scalar redLower2 = new Scalar(160, 80, 80);
    private final Scalar redUpper2 = new Scalar(180, 255, 255);
    
    // Detection parameters
    private final int MIN_BALL_AREA = 800;
    private final int MAX_BALL_AREA = 100000;
    
    // Results storage (thread-safe access)
    private volatile List<DetectedBall> lastDetectedBalls = new ArrayList<>();
    private volatile String[] lastPattern = new String[9];
    
    // Reusable Mats to avoid memory allocation every frame
    private Mat hsv = new Mat();
    private Mat redMask = new Mat();
    private Mat redMask1 = new Mat();
    private Mat redMask2 = new Mat();
    private Mat purpleMask = new Mat();
    private Mat greenMask = new Mat();
    private Mat silverMask = new Mat();
    private Mat kernel3 = new Mat();
    private Mat kernel5 = new Mat();
    private Mat kernel10 = new Mat();
    private Mat kernel15 = new Mat();
    private Mat kernel50 = new Mat();
    
    public WiffleBallPipeline() {
        // Pre-create kernels
        kernel3 = Imgproc.getStructuringElement(Imgproc.MORPH_RECT, new Size(3, 3));
        kernel5 = Imgproc.getStructuringElement(Imgproc.MORPH_RECT, new Size(5, 5));
        kernel10 = Imgproc.getStructuringElement(Imgproc.MORPH_RECT, new Size(10, 10));
        kernel15 = Imgproc.getStructuringElement(Imgproc.MORPH_RECT, new Size(15, 15));
        kernel50 = Imgproc.getStructuringElement(Imgproc.MORPH_RECT, new Size(50, 50));
        
        // Initialize pattern array
        for (int i = 0; i < 9; i++) {
            lastPattern[i] = "_";
        }
    }
    
    @Override
    public Mat processFrame(Mat input) {
        int frameWidth = input.cols();
        int frameHeight = input.rows();
        
        // Convert to HSV
        Imgproc.cvtColor(input, hsv, Imgproc.COLOR_RGB2HSV);
        
        // Detect red ramp area
        Mat rampMask = detectRedRampMask(hsv, frameHeight);
        
        // Detect color masks
        Core.inRange(hsv, purpleLower, purpleUpper, purpleMask);
        Imgproc.morphologyEx(purpleMask, purpleMask, Imgproc.MORPH_OPEN, kernel3);
        Imgproc.morphologyEx(purpleMask, purpleMask, Imgproc.MORPH_CLOSE, kernel5);
        
        Core.inRange(hsv, greenLower, greenUpper, greenMask);
        Imgproc.morphologyEx(greenMask, greenMask, Imgproc.MORPH_OPEN, kernel3);
        Imgproc.morphologyEx(greenMask, greenMask, Imgproc.MORPH_CLOSE, kernel5);
        
        // Detect silver bars for reflection filtering
        Core.inRange(hsv, silverLower, silverUpper, silverMask);
        Imgproc.morphologyEx(silverMask, silverMask, Imgproc.MORPH_CLOSE, kernel5);
        Mat silverExclusion = new Mat();
        Imgproc.dilate(silverMask, silverExclusion, kernel15);
        
        // Find balls
        List<DetectedBall> allBalls = new ArrayList<>();
        
        // Detect purple balls
        List<DetectedBall> purpleBalls = detectBallsFromMask(purpleMask, BallColor.PURPLE, 
                                                              rampMask, frameWidth);
        allBalls.addAll(purpleBalls);
        
        // Detect green balls
        List<DetectedBall> greenBalls = detectBallsFromMask(greenMask, BallColor.GREEN, 
                                                             rampMask, frameWidth);
        allBalls.addAll(greenBalls);
        
        // Filter out reflections on silver bars
        List<DetectedBall> filteredBalls = filterReflections(allBalls, silverExclusion, silverMask);
        
        // Merge balls split by end bar
        List<DetectedBall> mergedBalls = mergeSplitBalls(filteredBalls);
        
        // Sort by x position DESCENDING (right to left = ball 1 to ball 9)
        Collections.sort(mergedBalls, (a, b) -> Double.compare(b.center.x, a.center.x));
        
        // Limit to 9 balls max
        if (mergedBalls.size() > 9) {
            // Sort by area, keep largest 9, then re-sort by position
            Collections.sort(mergedBalls, (a, b) -> Double.compare(b.area, a.area));
            mergedBalls = new ArrayList<>(mergedBalls.subList(0, 9));
            Collections.sort(mergedBalls, (a, b) -> Double.compare(b.center.x, a.center.x));
        }
        
        // Store results
        lastDetectedBalls = mergedBalls;
        updatePattern(mergedBalls);
        
        // Draw debug overlay
        drawDebugOverlay(input, mergedBalls, rampMask);
        
        // Clean up
        silverExclusion.release();
        rampMask.release();
        
        return input;
    }
    
    /**
     * Detect the red ramp area mask
     */
    private Mat detectRedRampMask(Mat hsv, int frameHeight) {
        // Red wraps around in HSV
        Core.inRange(hsv, redLower1, redUpper1, redMask1);
        Core.inRange(hsv, redLower2, redUpper2, redMask2);
        Core.bitwise_or(redMask1, redMask2, redMask);
        
        // Clean up
        Imgproc.morphologyEx(redMask, redMask, Imgproc.MORPH_CLOSE, kernel10);
        Imgproc.morphologyEx(redMask, redMask, Imgproc.MORPH_OPEN, kernel10);
        
        // Find contours and keep only the main ramp
        List<MatOfPoint> contours = new ArrayList<>();
        Mat hierarchy = new Mat();
        Imgproc.findContours(redMask, contours, hierarchy, Imgproc.RETR_EXTERNAL, 
                            Imgproc.CHAIN_APPROX_SIMPLE);
        
        Mat rampMask = Mat.zeros(redMask.size(), CvType.CV_8UC1);
        
        // Filter for valid ramp contours
        for (MatOfPoint contour : contours) {
            double area = Imgproc.contourArea(contour);
            Rect bbox = Imgproc.boundingRect(contour);
            int centerY = bbox.y + bbox.height / 2;
            
            // Ramp should be large and in lower 75% of image
            if (area > 10000 && centerY > frameHeight * 0.25) {
                Imgproc.drawContours(rampMask, Collections.singletonList(contour), 
                                    -1, new Scalar(255), -1);
            }
        }
        
        // Dilate to include balls at edges
        Imgproc.dilate(rampMask, rampMask, kernel50);
        
        hierarchy.release();
        return rampMask;
    }
    
    /**
     * Detect balls from a color mask
     */
    private List<DetectedBall> detectBallsFromMask(Mat mask, BallColor color, 
                                                    Mat rampMask, int frameWidth) {
        List<DetectedBall> balls = new ArrayList<>();
        
        List<MatOfPoint> contours = new ArrayList<>();
        Mat hierarchy = new Mat();
        Imgproc.findContours(mask.clone(), contours, hierarchy, Imgproc.RETR_EXTERNAL, 
                            Imgproc.CHAIN_APPROX_SIMPLE);
        
        for (MatOfPoint contour : contours) {
            double blobArea = Imgproc.contourArea(contour);
            
            if (blobArea < MIN_BALL_AREA || blobArea > MAX_BALL_AREA) {
                continue;
            }
            
            // Check if ball touches red ramp
            if (!ballTouchesRamp(contour, rampMask)) {
                continue;
            }
            
            // Get bounding box
            Rect bbox = Imgproc.boundingRect(contour);
            int centerX = bbox.x + bbox.width / 2;
            
            // Aspect ratio check
            double aspectRatio = (double) bbox.width / Math.max(bbox.height, 1);
            
            // Position-based size expectation (perspective)
            double positionRatio = (double) centerX / frameWidth;
            double expectedSingleBallArea = 1500 + (positionRatio * 1500);
            
            // Try to split if elongated or too large
            boolean shouldTrySplit = (aspectRatio > 1.8) || (blobArea > expectedSingleBallArea * 1.8);
            
            if (!shouldTrySplit) {
                // Single ball
                Moments m = Imgproc.moments(contour);
                if (m.m00 > 0) {
                    int cx = (int) (m.m10 / m.m00);
                    int cy = (int) (m.m01 / m.m00);
                    balls.add(new DetectedBall(color, new Point(cx, cy), blobArea, contour));
                }
            } else {
                // Try to split using simple horizontal division for elongated blobs
                List<DetectedBall> splitBalls = trySplitBlob(contour, color, blobArea, bbox);
                balls.addAll(splitBalls);
            }
        }
        
        hierarchy.release();
        return balls;
    }
    
    /**
     * Try to split an elongated blob into multiple balls
     */
    private List<DetectedBall> trySplitBlob(MatOfPoint contour, BallColor color, 
                                             double blobArea, Rect bbox) {
        List<DetectedBall> balls = new ArrayList<>();
        
        // Estimate number of balls based on aspect ratio
        int numBalls = Math.max(1, (int) Math.round(bbox.width / (double) bbox.height));
        numBalls = Math.min(numBalls, 3); // Max 3 balls merged
        
        if (numBalls <= 1) {
            Moments m = Imgproc.moments(contour);
            if (m.m00 > 0) {
                int cx = (int) (m.m10 / m.m00);
                int cy = (int) (m.m01 / m.m00);
                balls.add(new DetectedBall(color, new Point(cx, cy), blobArea, contour));
            }
        } else {
            // Split horizontally into numBalls parts
            double segmentWidth = bbox.width / (double) numBalls;
            double areaPerBall = blobArea / numBalls;
            
            for (int i = 0; i < numBalls; i++) {
                int cx = bbox.x + (int) (segmentWidth * (i + 0.5));
                int cy = bbox.y + bbox.height / 2;
                balls.add(new DetectedBall(color, new Point(cx, cy), areaPerBall, contour));
            }
        }
        
        return balls;
    }
    
    /**
     * Check if a ball contour touches the red ramp area
     */
    private boolean ballTouchesRamp(MatOfPoint contour, Mat rampMask) {
        // Create mask for this ball
        Mat ballMask = Mat.zeros(rampMask.size(), CvType.CV_8UC1);
        Imgproc.drawContours(ballMask, Collections.singletonList(contour), -1, new Scalar(255), -1);
        
        // Dilate to catch balls near edges
        Mat dilatedBall = new Mat();
        Imgproc.dilate(ballMask, dilatedBall, kernel15);
        
        // Check overlap
        Mat overlap = new Mat();
        Core.bitwise_and(dilatedBall, rampMask, overlap);
        int overlapPixels = Core.countNonZero(overlap);
        
        ballMask.release();
        dilatedBall.release();
        overlap.release();
        
        return overlapPixels > 50;
    }
    
    /**
     * Filter out balls that are reflections on silver bars
     */
    private List<DetectedBall> filterReflections(List<DetectedBall> balls, 
                                                  Mat silverExclusion, Mat silverMask) {
        List<DetectedBall> filtered = new ArrayList<>();
        
        for (DetectedBall ball : balls) {
            int cx = (int) ball.center.x;
            int cy = (int) ball.center.y;
            
            // Check bounds
            if (cy < 0 || cy >= silverExclusion.rows() || cx < 0 || cx >= silverExclusion.cols()) {
                continue;
            }
            
            // Check if center is on silver
            double[] pixel = silverExclusion.get(cy, cx);
            if (pixel != null && pixel[0] > 0) {
                // Check overlap ratio with silver
                Mat ballMask = Mat.zeros(silverMask.size(), CvType.CV_8UC1);
                Imgproc.drawContours(ballMask, Collections.singletonList(ball.contour), 
                                    -1, new Scalar(255), -1);
                
                Mat overlap = new Mat();
                Core.bitwise_and(ballMask, silverMask, overlap);
                
                int ballPixels = Core.countNonZero(ballMask);
                int overlapPixels = Core.countNonZero(overlap);
                double overlapRatio = (double) overlapPixels / Math.max(ballPixels, 1);
                
                ballMask.release();
                overlap.release();
                
                // If more than 30% overlaps with silver, skip
                if (overlapRatio > 0.3) {
                    continue;
                }
            }
            
            filtered.add(ball);
        }
        
        return filtered;
    }
    
    /**
     * Merge balls that were split by the silver bar
     */
    private List<DetectedBall> mergeSplitBalls(List<DetectedBall> balls) {
        if (balls.size() <= 1) {
            return balls;
        }
        
        List<DetectedBall> merged = new ArrayList<>();
        boolean[] used = new boolean[balls.size()];
        
        // Sort by x position (right to left)
        List<Integer> sortedIndices = new ArrayList<>();
        for (int i = 0; i < balls.size(); i++) {
            sortedIndices.add(i);
        }
        Collections.sort(sortedIndices, (a, b) -> 
            Double.compare(balls.get(b).center.x, balls.get(a).center.x));
        
        for (int idx : sortedIndices) {
            if (used[idx]) continue;
            
            DetectedBall ball1 = balls.get(idx);
            List<DetectedBall> toMerge = new ArrayList<>();
            toMerge.add(ball1);
            used[idx] = true;
            
            for (int idx2 : sortedIndices) {
                if (used[idx2]) continue;
                
                DetectedBall ball2 = balls.get(idx2);
                
                // Only merge same color
                if (ball1.color != ball2.color) continue;
                
                // Dynamic merge distance based on ball size
                double avgArea = (ball1.area + ball2.area) / 2;
                double mergeDistance = 50 + (avgArea / 100);
                mergeDistance = Math.min(mergeDistance, 100);
                
                double distance = Math.sqrt(
                    Math.pow(ball1.center.x - ball2.center.x, 2) +
                    Math.pow(ball1.center.y - ball2.center.y, 2)
                );
                
                if (distance < mergeDistance) {
                    toMerge.add(ball2);
                    used[idx2] = true;
                }
            }
            
            if (toMerge.size() == 1) {
                merged.add(ball1);
            } else {
                // Merge multiple detections
                double totalArea = 0;
                double sumX = 0, sumY = 0;
                DetectedBall largest = toMerge.get(0);
                
                for (DetectedBall b : toMerge) {
                    totalArea += b.area;
                    sumX += b.center.x * b.area;
                    sumY += b.center.y * b.area;
                    if (b.area > largest.area) {
                        largest = b;
                    }
                }
                
                double avgX = sumX / totalArea;
                double avgY = sumY / totalArea;
                
                merged.add(new DetectedBall(ball1.color, new Point(avgX, avgY), 
                                           totalArea, largest.contour));
            }
        }
        
        return merged;
    }
    
    /**
     * Update the pattern array from detected balls
     */
    private void updatePattern(List<DetectedBall> balls) {
        // Reset pattern
        for (int i = 0; i < 9; i++) {
            lastPattern[i] = "_";
        }
        
        // Fill in detected balls
        for (int i = 0; i < Math.min(balls.size(), 9); i++) {
            lastPattern[i] = balls.get(i).color.getSymbol();
        }
    }
    
    /**
     * Draw debug overlay on the frame
     */
    private void drawDebugOverlay(Mat frame, List<DetectedBall> balls, Mat rampMask) {
        // Draw detected balls
        for (int i = 0; i < balls.size(); i++) {
            DetectedBall ball = balls.get(i);
            Point center = ball.center;
            
            Scalar circleColor;
            String label;
            
            if (ball.color == BallColor.PURPLE) {
                circleColor = new Scalar(255, 0, 255); // Magenta
                label = "P" + (i + 1);
            } else if (ball.color == BallColor.GREEN) {
                circleColor = new Scalar(0, 255, 0); // Green
                label = "G" + (i + 1);
            } else {
                circleColor = new Scalar(128, 128, 128); // Gray
                label = "?" + (i + 1);
            }
            
            Imgproc.circle(frame, center, 30, circleColor, 3);
            Imgproc.putText(frame, label, new Point(center.x - 15, center.y - 35),
                           Imgproc.FONT_HERSHEY_SIMPLEX, 0.6, new Scalar(255, 255, 255), 2);
        }
        
        // Draw pattern text
        String patternStr = String.join("", lastPattern);
        Imgproc.putText(frame, "Pattern: " + patternStr, new Point(10, 30),
                       Imgproc.FONT_HERSHEY_SIMPLEX, 0.7, new Scalar(255, 255, 255), 2);
        
        // Draw ball count
        Imgproc.putText(frame, "Count: " + balls.size(), new Point(10, 60),
                       Imgproc.FONT_HERSHEY_SIMPLEX, 0.7, new Scalar(255, 255, 255), 2);
    }
    
    // ========== PUBLIC GETTERS ==========
    
    /**
     * Get the last detected balls list
     */
    public List<DetectedBall> getDetectedBalls() {
        return new ArrayList<>(lastDetectedBalls);
    }
    
    /**
     * Get the last detected ball pattern as a string array
     * Index 0 = Ball 1 (rightmost/bottom of ramp)
     * Index 8 = Ball 9 (leftmost/top of ramp)
     */
    public String[] getPattern() {
        return lastPattern.clone();
    }
    
    /**
     * Get the pattern as a single string (e.g., "GPPPGPPPG")
     */
    public String getPatternString() {
        return String.join("", lastPattern);
    }
    
    /**
     * Get number of balls currently detected
     */
    public int getBallCount() {
        return lastDetectedBalls.size();
    }
    
    /**
     * Get the color of a specific ball position (1-9)
     */
    public String getBallColor(int position) {
        if (position < 1 || position > 9) return "_";
        return lastPattern[position - 1];
    }
    
    /**
     * Get colors in order as a readable string (e.g., "Purple, Green, Purple")
     */
    public String getColorsInOrder() {
        List<DetectedBall> balls = lastDetectedBalls;
        if (balls.isEmpty()) {
            return "No balls detected";
        }
        
        StringBuilder sb = new StringBuilder();
        for (int i = 0; i < balls.size(); i++) {
            if (i > 0) sb.append(", ");
            sb.append(balls.get(i).color.getName());
        }
        return sb.toString();
    }
    
    /**
     * Get count of each color
     */
    public int getPurpleCount() {
        int count = 0;
        for (DetectedBall ball : lastDetectedBalls) {
            if (ball.color == BallColor.PURPLE) count++;
        }
        return count;
    }
    
    public int getGreenCount() {
        int count = 0;
        for (DetectedBall ball : lastDetectedBalls) {
            if (ball.color == BallColor.GREEN) count++;
        }
        return count;
    }
    
    /**
     * Check if the detected pattern matches expected pattern
     */
    public boolean matchesPattern(String expected) {
        String detected = getPatternString();
        return detected.equals(expected);
    }
    
    /**
     * Get accuracy compared to expected pattern (0.0 to 1.0)
     */
    public double getAccuracy(String expected) {
        String detected = getPatternString();
        int matches = 0;
        int total = Math.min(detected.length(), expected.length());
        
        for (int i = 0; i < total; i++) {
            if (detected.charAt(i) == expected.charAt(i)) {
                matches++;
            }
        }
        
        return (double) matches / total;
    }
}
