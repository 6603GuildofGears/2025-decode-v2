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
            16.875, 19.25, 26.25, 33.25, 37.375, 47.5,
            54.375, 58.625, 66.375, 70.25, 77.125, 115.5, 127.5
    };

    private static final double[] RPM = {
            3125, 3000, 3125, 3250, 3325, 3325,
            3500, 3550, 3650, 3700, 3700, 4375, 4750
    };

    private static final double[] HOOD_POS = {
            0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
            0.05, 0.05, 0.55, 0.475, 0.475, 0.52, 0.5
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
