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
            50, 52.5, 53, 54.375, 55, 58.625, 60.25,
            62, 63.5, 65, 66, 68, 69,
            70.25, 72.875, 75.75, 77.125, 108.5,
            115.5, 120, 127.5
    };

    private static final double[] RPM = {
            3075, 2950, 3075, 3200, 3275, 3275,
            3275, 3300, 3400, 3450, 3450, 3500, 3500,
            3500, 3450, 3450, 3450, 3450, 3500,
            3650, 3650, 3650, 3650, 4150,
            4325, 4450, 4700
    };

    private static final double[] HOOD_POS = {
            0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
            0.05, 0.05, 0.05, 0.05, 0.05, 0.05, 0.05,
            0.1, 0.15, 0.25, 0.3, 0.3, 0.325,
            0.475, 0.475, 0.475, 0.475, 0.4875,
            0.5, 0.5, 0.5
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
