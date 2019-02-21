package org.firstinspires.ftc.teamcode.blackbox;

public class TimestampedReading<T> {
    public long time;
    public T value;

    public TimestampedReading(long timeIn, T valueIn) {
        time = timeIn;
        value = valueIn;
    }
}
