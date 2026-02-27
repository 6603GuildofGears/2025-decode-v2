package org.firstinspires.ftc.teamcode.pedroPathing.TeleOp;

/**
 * Lookup table for shooter tuning based on Limelight distance (inches).
 * Provides linear interpolation between points and clamps outside the range.
 */
public final class ShooterLookup {
    private ShooterLookup() {
        // Utility class
    }

    private static final double[] DIST_IN = {
            19.25, 21, 24.75, 29, 33, 35, 37, 40, 42,
            44, 47.125, 49, 50.75, 53, 56.5, 59, 61,
            62.25, 65, 65.75, 66.5, 68.5, 70, 71.5,
            74, 75.25, 76, 77.25, 78, 81, 83, 83.25,
            85, 86.75, 88.25, 90, 117.75, 123.5
    };

    private static final double[] RPM = {
            2800, 2800, 2800, 2800, 2850, 2850, 2850, 2850, 2850,
            2900, 2900, 2900, 2900, 3000, 3000, 3100, 3100,
            3125, 3150, 3200, 3200, 3225, 3225, 3250,
            3275, 3275, 3300, 3300, 3325, 3400, 3450, 3450,
            3500, 3500, 3550, 3550, 4000, 4500
    };

    private static final double[] HOOD_POS = {
            0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
            0.0, 0.1, 0.1, 0.1, 0.2, 0.25, 0.25, 0.25,
            0.25, 0.325, 0.325, 0.325, 0.35, 0.35, 0.35,
            0.35, 0.35, 0.35, 0.35, 0.35, 0.35, 0.35, 0.35,
            0.375, 0.39, 0.4, 0.4, 0.5, 0.5
    };

    public static Result lookup(double distanceInches) {
        if (distanceInches <= DIST_IN[0]) {
            return new Result(RPM[0], HOOD_POS[0]);
        }
        int last = DIST_IN.length - 1;
        if (distanceInches >= DIST_IN[last]) {
            return new Result(RPM[last], HOOD_POS[last]);
        }

        for (int i = 0; i < last; i++) {
            double d0 = DIST_IN[i];
            double d1 = DIST_IN[i + 1];
            if (distanceInches >= d0 && distanceInches <= d1) {
                double t = (distanceInches - d0) / (d1 - d0);
                double rpm = lerp(RPM[i], RPM[i + 1], t);
                double hood = lerp(HOOD_POS[i], HOOD_POS[i + 1], t);
                return new Result(rpm, hood);
            }
        }

        return new Result(RPM[last], HOOD_POS[last]);
    }

    private static double lerp(double a, double b, double t) {
        return a + (b - a) * t;
    }

    public static final class Result {
        public final double rpm;
        public final double hoodPos;

        public Result(double rpm, double hoodPos) {
            this.rpm = rpm;
            this.hoodPos = hoodPos;
        }
    }
}
