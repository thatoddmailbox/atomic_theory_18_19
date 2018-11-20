package org.firstinspires.ftc.teamcode.utils;

public class PersistentHeading {
    private static double _savedHeading = Double.POSITIVE_INFINITY;

    public static void clearSavedHeading() {
        _savedHeading = Double.POSITIVE_INFINITY;
    }

    public static void saveHeading(double heading) {
        _savedHeading = heading;
    }

    public static boolean haveSavedHeading() {
        return (_savedHeading != Double.POSITIVE_INFINITY);
    }

    public static double getSavedHeading() {
        return _savedHeading;
    }
}
