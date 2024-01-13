package org.firstinspires.ftc.teamcode.math;

public class otherUtils {
    public double ensureRange(double value, double min, double max) {
        return Math.min(Math.max(value, min), max);
    }
}
