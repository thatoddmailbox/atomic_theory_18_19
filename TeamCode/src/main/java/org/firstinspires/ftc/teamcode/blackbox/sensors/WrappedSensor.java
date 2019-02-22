package org.firstinspires.ftc.teamcode.blackbox.sensors;

import org.firstinspires.ftc.teamcode.blackbox.Datastreamable;

public abstract class WrappedSensor<T> implements Datastreamable {
    String _name;
    T _sensor;

    public WrappedSensor(T sensor, String name) throws InterruptedException {
        _name = name;
        _sensor = sensor;
    }

    @Override
    public String getName() {
        return _name;
    }
}
