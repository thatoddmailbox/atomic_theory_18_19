package org.firstinspires.ftc.teamcode.blackbox;

import java.util.concurrent.LinkedBlockingQueue;

public class Datastream<T> {
    public String name;
    public LinkedBlockingQueue<TimestampedReading<T>> data;
    public long lastMillis = 0;

    public Datastream(String nameIn) {
        name = nameIn;
        data = new LinkedBlockingQueue<TimestampedReading<T>>();
    }

    public void storeReading(T value) {
        long newMillis = System.currentTimeMillis();
        if (lastMillis == newMillis) {
            return;
        }
        lastMillis = newMillis;
        data.add(new TimestampedReading<T>(newMillis, value));
    }
}
