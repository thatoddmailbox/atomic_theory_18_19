package org.firstinspires.ftc.teamcode.blackbox.sensors;

import com.qualcomm.hardware.bosch.BNO055IMU;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.teamcode.blackbox.Datastream;

public class WrappedBNO055IMU extends WrappedSensor<BNO055IMU> {
    public WrappedBNO055IMU(BNO055IMU sensor, String name) throws InterruptedException {
        super(sensor, name);

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

        };
    }

    public boolean isGyroCalibrated() {
        return _sensor.isGyroCalibrated();
    }

    public Orientation getAngularOrientation(AxesReference extrinsic, AxesOrder zyx, AngleUnit degrees) {
        return _sensor.getAngularOrientation(extrinsic, zyx, degrees);
    }
}
