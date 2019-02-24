package org.firstinspires.ftc.teamcode.blackbox.sensors;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cRangeSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import java.lang.reflect.Constructor;
import java.lang.reflect.InvocationTargetException;
import java.util.HashMap;

public class SensorFactory {
    private static HashMap<Class, Class<? extends WrappedSensor>> _mapping = new HashMap<Class, Class<? extends WrappedSensor>>();

    static {
        _mapping.put(ModernRoboticsI2cRangeSensor.class, WrappedMRRangeSensor.class);
        _mapping.put(BNO055IMU.class, WrappedBNO055IMU.class);
        _mapping.put(LynxModule.class, WrappedLynxModule.class);
    }

    public static <T extends WrappedSensor<Q>, Q> T getSensor(HardwareMap hwMap, Class<Q> sensorClass, String niceName, String configName) {
        Q unwrappedSensor = hwMap.get(sensorClass, configName);
        Class<? extends WrappedSensor> wrappedSensorClass = _mapping.get(sensorClass);
        try {
            Constructor<WrappedSensor> constructor = (Constructor<WrappedSensor>) wrappedSensorClass.getConstructor(sensorClass, String.class);
            return (T) constructor.newInstance(unwrappedSensor, niceName);
        } catch (NoSuchMethodException | IllegalAccessException | InvocationTargetException | InstantiationException e) {
            e.printStackTrace();
        }
        return null;
    }
}
