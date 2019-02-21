package org.firstinspires.ftc.teamcode.robolog;

public class TimestampedReading<T> {
    public long time;
    public T value;

    public TimestampedReading(long timeIn, T valueIn) {
        time = timeIn;
        value = valueIn;
    }
}
