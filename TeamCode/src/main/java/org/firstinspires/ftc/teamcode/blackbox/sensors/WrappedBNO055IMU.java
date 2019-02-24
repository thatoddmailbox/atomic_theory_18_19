package org.firstinspires.ftc.teamcode.blackbox.sensors;

import com.qualcomm.hardware.bosch.BNO055IMU;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.teamcode.blackbox.Datastream;

public class WrappedBNO055IMU extends WrappedSensor<BNO055IMU> {
    private Datastream<Float> _xAngleRawStream;
    private Datastream<Float> _yAngleRawStream;
    private Datastream<Float> _zAngleRawStream;
    private Datastream<Double> _zAngleOffsetStream;

    public float headingOffset = 0;

    public WrappedBNO055IMU(BNO055IMU sensor, String name) throws InterruptedException {
        super(sensor, name);

        _xAngleRawStream = new Datastream<Float>("xAngle (raw)");
        _yAngleRawStream = new Datastream<Float>("yAngle (raw)");
        _zAngleRawStream = new Datastream<Float>("zAngle (raw)");
        _zAngleOffsetStream = new Datastream<Double>("zAngle (offset)");

        BNO055IMU.Parameters imuParameters = new BNO055IMU.Parameters();

        imuParameters.mode                = BNO055IMU.SensorMode.NDOF;
        imuParameters.angleUnit           = BNO055IMU.AngleUnit.DEGREES;
        imuParameters.accelUnit           = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        imuParameters.loggingEnabled      = false;

        sensor.initialize(imuParameters);

        // axis remap? maybe?
        Thread.sleep(100);
        sensor.write8(BNO055IMU.Register.OPR_MODE, BNO055IMU.SensorMode.CONFIG.bVal);
        Thread.sleep(100);
        sensor.write8(BNO055IMU.Register.AXIS_MAP_CONFIG, 0x24);
        sensor.write8(BNO055IMU.Register.AXIS_MAP_SIGN, 0x05);
        sensor.write8(BNO055IMU.Register.OPR_MODE, BNO055IMU.SensorMode.IMU.bVal);
        Thread.sleep(100);
    }

    @Override
    public Datastream[] getDatastreams() {
        return new Datastream[] {
                _xAngleRawStream,
                _yAngleRawStream,
                _zAngleRawStream,
                _zAngleOffsetStream
        };
    }

    public boolean isGyroCalibrated() {
        return _sensor.isGyroCalibrated();
    }

    private Orientation getAngularOrientationInternal(AxesReference extrinsic, AxesOrder zyx, AngleUnit degrees) {
        return _sensor.getAngularOrientation(extrinsic, AxesOrder.ZYX, degrees);
    }

    @Deprecated
    public Orientation getAngularOrientation(AxesReference extrinsic, AxesOrder zyx, AngleUnit degrees) {
        return getAngularOrientationInternal(extrinsic, zyx, degrees);
    }

    public float getRawXAngle() {
        Orientation orientation = getAngularOrientationInternal(AxesReference.EXTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        _xAngleRawStream.storeReading(orientation.thirdAngle);
        return orientation.thirdAngle;
    }

    public float getRawYAngle() {
        Orientation orientation = getAngularOrientationInternal(AxesReference.EXTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        _yAngleRawStream.storeReading(orientation.secondAngle);
        return orientation.secondAngle;
    }

    public float getRawZAngle() {
        Orientation orientation = getAngularOrientationInternal(AxesReference.EXTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        _zAngleRawStream.storeReading(orientation.firstAngle);
        return orientation.firstAngle;
    }

    public void resetHeading() {
        headingOffset = getRawZAngle();
    }

    private double getHeadingInternal() {
        double angle = getRawZAngle();
        double sign = Math.signum(headingOffset);
        if (headingOffset > 0) {
            if (angle > -180 && angle < headingOffset - 180){
                return 360 - Math.abs(angle) - headingOffset;
            } else {
                return angle - headingOffset;
            }
        } else {
            if (angle < 180 && angle > headingOffset + 180){
                return -(360 - Math.abs(angle) + headingOffset);
            } else {
                return angle - headingOffset;
            }
        }
    }

    public double getHeading() {
        double heading = getHeadingInternal();
        _zAngleOffsetStream.storeReading(heading);
        return heading;
    }
}
