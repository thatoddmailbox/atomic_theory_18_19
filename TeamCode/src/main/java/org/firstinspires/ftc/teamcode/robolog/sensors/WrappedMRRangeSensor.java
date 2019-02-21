package org.firstinspires.ftc.teamcode.robolog.sensors;

import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cRangeSensor;

import org.firstinspires.ftc.teamcode.robolog.Datastream;
import org.firstinspires.ftc.teamcode.robolog.Datastreamable;

public class WrappedMRRangeSensor implements Datastreamable {
    public Datastream<Double> cmOpticalStream;
    public Datastream<Double> cmUltrasonicStream;

    public boolean loggingEnabled = false;

    private String _name;
    private ModernRoboticsI2cRangeSensor _range;

    public WrappedMRRangeSensor(ModernRoboticsI2cRangeSensor range, String name) {
        cmOpticalStream = new Datastream<Double>("cmOptical");
        cmUltrasonicStream = new Datastream<Double>(" cmUltrasonic");
        _name = name;
        _range = range;
    }

    @Override
    public String getName() {
        return _name;
    }

    @Override
    public Datastream[] getDatastreams() {
        return new Datastream[] {
                cmOpticalStream,
                cmUltrasonicStream
        };
    }

    public double cmOptical() {
        double reading = _range.cmOptical();
        if (loggingEnabled) {
            cmOpticalStream.storeReading(reading);
        }
        return reading;
    }

    public double cmUltrasonic() {
        double reading = _range.cmUltrasonic();
        if (loggingEnabled) {
            cmUltrasonicStream.storeReading(reading);
        }
        return reading;
    }
}
