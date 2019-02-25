package org.firstinspires.ftc.teamcode.blackbox;

import android.graphics.Bitmap;

public class TimestampedReading<T> {
    public long time;
    public T value;
    public Bitmap image;

    public TimestampedReading(long timeIn, T valueIn) {
        time = timeIn;
        value = valueIn;
        image = null;
    }

    public TimestampedReading(long timeIn, T valueIn, Bitmap imageIn) {
        time = timeIn;
        value = valueIn;
        image = imageIn;
    }
}
