package org.firstinspires.ftc.teamcode.robolog.sensors;

import org.firstinspires.ftc.teamcode.robolog.Datastream;
import org.firstinspires.ftc.teamcode.robolog.Datastreamable;

public abstract class WrappedSensor<T> implements Datastreamable {
    String _name;
    T _sensor;

    public WrappedSensor(T sensor, String name) {
        _name = name;
        _sensor = sensor;
    }

    @Override
    public String getName() {
        return _name;
    }
}
