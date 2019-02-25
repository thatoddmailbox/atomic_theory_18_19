package org.firstinspires.ftc.teamcode.blackbox;

import android.graphics.Bitmap;

import java.lang.reflect.ParameterizedType;
import java.util.concurrent.LinkedBlockingQueue;

public class Datastream<T> {
    public String name;
    public LinkedBlockingQueue<TimestampedReading<T>> data;
    public long lastMillis = 0;

    private boolean _attachedImages = false;

    public Datastream(String nameIn) {
        name = nameIn;
        data = new LinkedBlockingQueue<TimestampedReading<T>>();
    }

    public void enableAttachedImages() {
        _attachedImages = true;
    }

    public boolean hasAttachedImages() {
        return _attachedImages;
    }

    public void storeReading(T value) {
        storeReading(value, null);
    }

    public void storeReading(T value, Bitmap image) {
        long newMillis = System.currentTimeMillis();
        if (lastMillis == newMillis) {
            return;
        }
        lastMillis = newMillis;
        data.add(new TimestampedReading<T>(newMillis, value, image));
    }
}
