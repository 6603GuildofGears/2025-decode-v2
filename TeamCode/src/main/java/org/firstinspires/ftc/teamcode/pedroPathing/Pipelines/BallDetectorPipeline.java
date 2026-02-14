package org.firstinspires.ftc.teamcode.pedroPathing.Pipelines;

import android.content.res.AssetFileDescriptor;
import android.content.res.AssetManager;
import android.graphics.Canvas;
import android.graphics.Color;
import android.graphics.Paint;
import android.util.Size;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.internal.camera.calibration.CameraCalibration;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.VisionProcessor;

import org.opencv.core.Mat;
import org.opencv.imgproc.Imgproc;

import org.tensorflow.lite.Interpreter;

import java.io.FileInputStream;
import java.io.IOException;
import java.nio.ByteBuffer;
import java.nio.ByteOrder;
import java.nio.MappedByteBuffer;
import java.nio.channels.FileChannel;
import java.util.ArrayList;
import java.util.Collections;
import java.util.Comparator;
import java.util.List;

/**
 * Ball Detection Pipeline for FTC DECODE
 *
 * Uses a custom YOLOv8n TFLite model to detect green and purple balls on the ramp.
 * Implements VisionProcessor for SDK 11.x (no TFOD dependency).
 *
 * Returns balls sorted by Y position (lowest/bottom first) for ramp order.
 *
 * Usage:
 *   BallDetectorPipeline.init(opMode, "logitech");
 *   // In loop:
 *   BallDetectorPipeline.pollOnce();          // kept for API compat (no-op)
 *   List<BallDetection> balls = BallDetectorPipeline.getDetectedBalls();
 */
public class BallDetectorPipeline implements VisionProcessor {

    // ── Model configuration ──
    private static final String   MODEL_ASSET    = "ftc_ball_nano_320.tflite";
    private static final String[] LABELS         = {"green_ball", "purple_ball"};
    private static final int      MODEL_INPUT    = 320;
    private static final float    MIN_CONF_DEF   = 0.5f;

    // ── Instance state ──
    private Interpreter  interpreter;
    private int          frameWidth;
    private int          frameHeight;
    private float        minConfidence = MIN_CONF_DEF;

    // Buffers (reused every frame)
    private ByteBuffer inputBuffer;
    private ByteBuffer outputBuffer;
    private int outRows;   // 4 + numClasses
    private int outCols;   // number of candidate detections

    // Latest results (written on camera thread, read on OpMode thread)
    private volatile List<BallDetection> latestDetections = new ArrayList<>();

    // Compute-time tracking (ms per processFrame call)
    private volatile double lastComputeMs = 0.0;

    // ── Singleton + VisionPortal ──
    private static BallDetectorPipeline instance;
    private static VisionPortal visionPortal;

    // ── Drawing paints ──
    private final Paint greenPaint  = new Paint();
    private final Paint purplePaint = new Paint();
    private final Paint textPaint   = new Paint();

    // ====================================================================
    //  BallDetection data class  (public API unchanged)
    // ====================================================================

    public static class BallDetection {
        public final String  label;
        public final boolean isGreen;
        public final float   confidence;
        public final float   centerX;
        public final float   centerY;
        public final float   width;
        public final float   height;
        public final float   bottomY;
        public final int     rampPosition;

        public BallDetection(String label, float confidence,
                             float left, float top, float right, float bottom,
                             int position) {
            this.label      = label;
            this.isGreen    = label.equals("green_ball");
            this.confidence = confidence;
            this.centerX    = (left + right) / 2f;
            this.centerY    = (top + bottom) / 2f;
            this.width      = right - left;
            this.height     = bottom - top;
            this.bottomY    = bottom;
            this.rampPosition = position;
        }

        @Override
        public String toString() {
            return String.format("%s #%d (%.0f%%) at (%.0f, %.0f)",
                    isGreen ? "GREEN" : "PURPLE",
                    rampPosition,
                    confidence * 100,
                    centerX, centerY);
        }
    }

    // ====================================================================
    //  Public static API
    // ====================================================================

    /** Initialise with default webcam name "Webcam 1". */
    public static void init(OpMode opMode) {
        init(opMode, "Webcam 1");
    }

    /** Initialise with a specific webcam name. */
    public static void init(OpMode opMode, String webcamName) {
        instance = new BallDetectorPipeline();
        instance.loadModel(opMode);

        visionPortal = new VisionPortal.Builder()
                .setCamera(opMode.hardwareMap.get(WebcamName.class, webcamName))
                .setCameraResolution(new Size(640, 480))
                .addProcessor(instance)
                .setStreamFormat(VisionPortal.StreamFormat.MJPEG)
                .enableLiveView(true)
                .setAutoStopLiveView(true)
                .build();

        opMode.telemetry.addData("Ball Detector", "Initialized (VisionProcessor)");
        opMode.telemetry.addData("Model", MODEL_ASSET);
        opMode.telemetry.update();
    }

    /** Kept for API compatibility — processing happens on the camera thread. */
    public static void pollOnce() { /* no-op */ }

    public static List<BallDetection> getDetectedBalls() {
        if (instance == null) return new ArrayList<>();
        return new ArrayList<>(instance.latestDetections);
    }

    public static List<BallDetection> getGreenBalls() {
        List<BallDetection> out = new ArrayList<>();
        for (BallDetection b : getDetectedBalls()) if (b.isGreen) out.add(b);
        return out;
    }

    public static List<BallDetection> getPurpleBalls() {
        List<BallDetection> out = new ArrayList<>();
        for (BallDetection b : getDetectedBalls()) if (!b.isGreen) out.add(b);
        return out;
    }

    public static String getBallSequence() {
        StringBuilder sb = new StringBuilder();
        for (BallDetection b : getDetectedBalls()) sb.append(b.isGreen ? "G" : "P");
        return sb.toString();
    }

    public static BallDetection getFirstBall() {
        List<BallDetection> balls = getDetectedBalls();
        return balls.isEmpty() ? null : balls.get(0);
    }

    public static boolean hasBalls()  { return !getDetectedBalls().isEmpty(); }
    public static int     getBallCount() { return getDetectedBalls().size(); }

    public static void setMinConfidence(float c) {
        if (instance != null) instance.minConfidence = c;
    }

    public static void setLiveViewEnabled(boolean on) {
        if (visionPortal == null) return;
        if (on) visionPortal.resumeLiveView(); else visionPortal.stopLiveView();
    }

    public static void stop() {
        if (visionPortal != null) { visionPortal.close(); visionPortal = null; }
        if (instance != null && instance.interpreter != null) {
            instance.interpreter.close();
            instance.interpreter = null;
        }
        instance = null;
    }

    /** Returns the time in ms that the last processFrame() call took. */
    public static double getComputeMs() {
        return instance != null ? instance.lastComputeMs : 0.0;
    }

    public static boolean isReady() {
        return visionPortal != null &&
               visionPortal.getCameraState() == VisionPortal.CameraState.STREAMING;
    }

    // ====================================================================
    //  VisionProcessor implementation
    // ====================================================================

    @Override
    public void init(int width, int height, CameraCalibration calibration) {
        frameWidth  = width;
        frameHeight = height;

        greenPaint.setColor(Color.GREEN);
        greenPaint.setStyle(Paint.Style.STROKE);
        greenPaint.setStrokeWidth(4);

        purplePaint.setColor(Color.rgb(180, 0, 255));
        purplePaint.setStyle(Paint.Style.STROKE);
        purplePaint.setStrokeWidth(4);

        textPaint.setColor(Color.WHITE);
        textPaint.setTextSize(28);
        textPaint.setAntiAlias(true);
    }

    // Reusable byte array for bulk pixel read (allocated once)
    private byte[] pixelBytes;

    @Override
    public Object processFrame(Mat frame, long captureTimeNanos) {
        if (interpreter == null) return null;
        long t0 = System.nanoTime();

        // ── 1. Pre-process: resize → RGB → bulk read → normalise ──
        Mat rgb = new Mat();
        Imgproc.cvtColor(frame, rgb, Imgproc.COLOR_RGBA2RGB);

        Mat resized = new Mat();
        Imgproc.resize(rgb, resized, new org.opencv.core.Size(MODEL_INPUT, MODEL_INPUT));
        rgb.release();

        // Bulk read all pixels in one JNI call
        int totalBytes = MODEL_INPUT * MODEL_INPUT * 3;
        if (pixelBytes == null || pixelBytes.length != totalBytes) {
            pixelBytes = new byte[totalBytes];
        }
        resized.get(0, 0, pixelBytes);
        resized.release();

        // Fill input buffer from byte array (pure Java — fast)
        inputBuffer.rewind();
        for (int i = 0; i < totalBytes; i++) {
            inputBuffer.putFloat((pixelBytes[i] & 0xFF) / 255.0f);
        }

        // ── 2. Run inference ──
        outputBuffer.rewind();
        interpreter.run(inputBuffer, outputBuffer);
        outputBuffer.rewind();

        // ── 3. Post-process (read output buffer directly) ──
        int numClasses = LABELS.length;   // 2
        float scaleX = (float) frameWidth  / MODEL_INPUT;
        float scaleY = (float) frameHeight / MODEL_INPUT;
        int stride = outCols;

        List<BallDetection> detections = new ArrayList<>();

        for (int d = 0; d < outCols; d++) {
            float cx = outputBuffer.getFloat((0 * stride + d) * 4);
            float cy = outputBuffer.getFloat((1 * stride + d) * 4);
            float w  = outputBuffer.getFloat((2 * stride + d) * 4);
            float h  = outputBuffer.getFloat((3 * stride + d) * 4);

            // Best class
            int   bestIdx  = 0;
            float bestConf = 0f;
            for (int c = 0; c < numClasses; c++) {
                float conf = outputBuffer.getFloat(((4 + c) * stride + d) * 4);
                if (conf > bestConf) { bestConf = conf; bestIdx = c; }
            }
            if (bestConf < minConfidence) continue;

            float left   = (cx - w / 2f) * scaleX;
            float top    = (cy - h / 2f) * scaleY;
            float right  = (cx + w / 2f) * scaleX;
            float bottom = (cy + h / 2f) * scaleY;

            detections.add(new BallDetection(
                    LABELS[bestIdx], bestConf, left, top, right, bottom, 0));
        }

        // ── 4. NMS ──
        detections = nms(detections, 0.45f);

        // ── 5. Sort by bottomY descending (bottom of ramp first) ──
        Collections.sort(detections, new Comparator<BallDetection>() {
            @Override public int compare(BallDetection a, BallDetection b) {
                return Float.compare(b.bottomY, a.bottomY);
            }
        });

        // Assign ramp positions 1, 2, 3 ...
        List<BallDetection> positioned = new ArrayList<>();
        int pos = 1;
        for (BallDetection det : detections) {
            positioned.add(new BallDetection(det.label, det.confidence,
                    det.centerX - det.width / 2f, det.centerY - det.height / 2f,
                    det.centerX + det.width / 2f, det.centerY + det.height / 2f,
                    pos++));
        }

        latestDetections = positioned;
        lastComputeMs = (System.nanoTime() - t0) / 1_000_000.0;
        return positioned;   // forwarded to onDrawFrame
    }

    @SuppressWarnings("unchecked")
    @Override
    public void onDrawFrame(Canvas canvas, int onscreenWidth, int onscreenHeight,
                            float scaleBmpPxToCanvasPx, float scaleCanvasDensity,
                            Object userContext) {
        if (userContext == null) return;
        List<BallDetection> dets = (List<BallDetection>) userContext;

        for (BallDetection det : dets) {
            Paint p = det.isGreen ? greenPaint : purplePaint;
            p.setStrokeWidth(3 * scaleCanvasDensity);
            textPaint.setTextSize(24 * scaleCanvasDensity);

            float l = (det.centerX - det.width  / 2f) * scaleBmpPxToCanvasPx;
            float t = (det.centerY - det.height / 2f) * scaleBmpPxToCanvasPx;
            float r = (det.centerX + det.width  / 2f) * scaleBmpPxToCanvasPx;
            float b = (det.centerY + det.height / 2f) * scaleBmpPxToCanvasPx;

            canvas.drawRect(l, t, r, b, p);
            canvas.drawText(
                    String.format("#%d %s %.0f%%", det.rampPosition,
                            det.isGreen ? "G" : "P", det.confidence * 100),
                    l, t - 6 * scaleCanvasDensity, textPaint);
        }
    }

    // ====================================================================
    //  Internal helpers
    // ====================================================================

    private void loadModel(OpMode opMode) {
        try {
            AssetManager am = opMode.hardwareMap.appContext.getAssets();
            AssetFileDescriptor afd = am.openFd(MODEL_ASSET);
            FileInputStream fis = new FileInputStream(afd.getFileDescriptor());
            FileChannel channel = fis.getChannel();
            MappedByteBuffer modelBuffer = channel.map(
                    FileChannel.MapMode.READ_ONLY,
                    afd.getStartOffset(), afd.getDeclaredLength());

            Interpreter.Options opts = new Interpreter.Options();
            opts.setNumThreads(2);
            interpreter = new Interpreter(modelBuffer, opts);

            // Input buffer: 1 × 320 × 320 × 3 × 4 bytes
            inputBuffer = ByteBuffer.allocateDirect(MODEL_INPUT * MODEL_INPUT * 3 * 4);
            inputBuffer.order(ByteOrder.nativeOrder());

            // Output buffer: shape [1, 4+numClasses, numDetections]
            int[] outShape = interpreter.getOutputTensor(0).shape();
            outRows = outShape[1];   // 4 + numClasses (6)
            outCols = outShape[2];   // e.g. 8400
            outputBuffer = ByteBuffer.allocateDirect(outRows * outCols * 4);
            outputBuffer.order(ByteOrder.nativeOrder());

        } catch (IOException e) {
            throw new RuntimeException("Failed to load TFLite model: " + e.getMessage(), e);
        }
    }

    // ── Simple greedy NMS ──
    private static List<BallDetection> nms(List<BallDetection> dets, float iouThresh) {
        List<BallDetection> sorted = new ArrayList<>(dets);
        Collections.sort(sorted, new Comparator<BallDetection>() {
            @Override public int compare(BallDetection a, BallDetection b) {
                return Float.compare(b.confidence, a.confidence);
            }
        });

        List<BallDetection> keep = new ArrayList<>();
        boolean[] suppressed = new boolean[sorted.size()];

        for (int i = 0; i < sorted.size(); i++) {
            if (suppressed[i]) continue;
            keep.add(sorted.get(i));
            for (int j = i + 1; j < sorted.size(); j++) {
                if (!suppressed[j] && iou(sorted.get(i), sorted.get(j)) > iouThresh) {
                    suppressed[j] = true;
                }
            }
        }
        return keep;
    }

    private static float iou(BallDetection a, BallDetection b) {
        float aL = a.centerX - a.width / 2f, aR = a.centerX + a.width / 2f;
        float aT = a.centerY - a.height / 2f, aB = a.centerY + a.height / 2f;
        float bL = b.centerX - b.width / 2f, bR = b.centerX + b.width / 2f;
        float bT = b.centerY - b.height / 2f, bB = b.centerY + b.height / 2f;

        float interW = Math.max(0, Math.min(aR, bR) - Math.max(aL, bL));
        float interH = Math.max(0, Math.min(aB, bB) - Math.max(aT, bT));
        float inter   = interW * interH;
        float union   = a.width * a.height + b.width * b.height - inter;
        return union > 0 ? inter / union : 0;
    }
}
