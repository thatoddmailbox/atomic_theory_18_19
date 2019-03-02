package org.firstinspires.ftc.teamcode.blackbox.sensors;

import com.qualcomm.hardware.lynx.LynxNackException;
import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cRangeSensor;

import org.firstinspires.ftc.teamcode.blackbox.Datastream;
import org.firstinspires.ftc.teamcode.hardware.MuxedRangeSensor;

public class WrappedMuxedRangeSensor extends WrappedSensor<MuxedRangeSensor> {
    public Datastream<Double> cmOpticalStream;
    public Datastream<Double> cmUltrasonicStream;

    public WrappedMuxedRangeSensor(MuxedRangeSensor range, String name) throws InterruptedException {
        super(range, name);
        cmOpticalStream = new Datastream<Double>("cmOptical");
        cmUltrasonicStream = new Datastream<Double>("cmUltrasonic");
    }

    @Override
    public Datastream[] getDatastreams() {
        return new Datastream[] {
                cmOpticalStream,
                cmUltrasonicStream
        };
    }

    public double cmOptical() {
        double reading;
        try {
            reading = _sensor.cmOptical();
        } catch (LynxNackException | InterruptedException e) {
            cmOpticalStream.storeReading(254.0);
            e.printStackTrace();
            return 255;
        }
        cmOpticalStream.storeReading(reading);
        return reading;
    }

    public double cmUltrasonic() {
        double reading;
        try {
            reading = _sensor.cmUltrasonic();
        } catch (LynxNackException | InterruptedException e) {
            cmUltrasonicStream.storeReading(254.0);
            e.printStackTrace();
            return 255;
        }
        cmUltrasonicStream.storeReading(reading);
        return reading;
    }
}
