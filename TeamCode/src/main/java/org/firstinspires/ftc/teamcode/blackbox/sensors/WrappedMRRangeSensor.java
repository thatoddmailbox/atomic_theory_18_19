package org.firstinspires.ftc.teamcode.blackbox.sensors;

import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cRangeSensor;

import org.firstinspires.ftc.teamcode.blackbox.Datastream;

public class WrappedMRRangeSensor extends WrappedSensor<ModernRoboticsI2cRangeSensor> {
    public Datastream<Double> cmOpticalStream;
    public Datastream<Double> cmUltrasonicStream;

    public boolean loggingEnabled = false;

    public WrappedMRRangeSensor(ModernRoboticsI2cRangeSensor range, String name) {
        super(range, name);
        cmOpticalStream = new Datastream<Double>("cmOptical");
        cmUltrasonicStream = new Datastream<Double>(" cmUltrasonic");
    }

    @Override
    public Datastream[] getDatastreams() {
        return new Datastream[] {
                cmOpticalStream,
                cmUltrasonicStream
        };
    }

    public double cmOptical() {
        double reading = _sensor.cmOptical();
        if (loggingEnabled) {
            cmOpticalStream.storeReading(reading);
        }
        return reading;
    }

    public double cmUltrasonic() {
        double reading = _sensor.cmUltrasonic();
        if (loggingEnabled) {
            cmUltrasonicStream.storeReading(reading);
        }
        return reading;
    }
}
