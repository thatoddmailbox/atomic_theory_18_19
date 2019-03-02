package org.firstinspires.ftc.teamcode.hardware;

import android.content.Context;

import com.qualcomm.hardware.lynx.LynxI2cDeviceSynch;
import com.qualcomm.hardware.lynx.LynxI2cDeviceSynchV2;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.hardware.lynx.LynxNackException;
import com.qualcomm.hardware.lynx.commands.core.LynxI2cWriteReadMultipleBytesCommand;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.I2cAddr;

public class UltrasonicHub {
    private static final byte ADDRESS_ULTRASONIC_DEFAULT = 0x28;

    private static final byte ADDRESS_ULTRASONIC_BROADCAST_0 = 0x30;
    private static final byte ADDRESS_ULTRASONIC_BROADCAST_STEP = 0x02;

    private static final byte REGISTER_ULTRASONIC_READING = 0x04;
    private static final byte REGISTER_OPTICAL_READING = 0x05;

    private LynxModule _hub;
    private int _hubPort;

    private I2cAddr _ultrasonicAddress;

    private TCA9545A _mux;
    private LynxI2cDeviceSynch _ultrasonicSensor;

    private boolean _broadcastMode = false;

    private LynxI2cDeviceSynch[] _ultrasonicBroadcastSensors;

    public UltrasonicHub(Context context, LynxModule hub, int hubPort, DigitalChannel muxReset) {
        if (!hub.isCommandSupported(LynxI2cWriteReadMultipleBytesCommand.class)) {
            throw new RuntimeException("Expansion hub firmware requires update! LynxI2cWriteReadMultipleBytesCommand not supported!");
        }

        _hub = hub;
        _hubPort = hubPort;
        _ultrasonicAddress = I2cAddr.create8bit(ADDRESS_ULTRASONIC_DEFAULT);
        _broadcastMode = false;

        // 0x30 = front left
        // 0x32 = front right
        // 0x34 = back right
        // 0x36 = back left

        _mux = new TCA9545A(_hub, _hubPort, muxReset, false, false);
        _ultrasonicSensor = new LynxI2cDeviceSynchV2(context, _hub, _hubPort);
        _ultrasonicSensor.setI2cAddr(_ultrasonicAddress);
    }

    public void enableBroadcastMode(Context context) {
        _broadcastMode = true;
        _ultrasonicBroadcastSensors = new LynxI2cDeviceSynchV2[4];

        for (int i = 0; i < 4; i++) {
            _ultrasonicBroadcastSensors[i] = new LynxI2cDeviceSynchV2(context, _hub, _hubPort);
            _ultrasonicBroadcastSensors[i].setI2cAddr(I2cAddr.create8bit(ADDRESS_ULTRASONIC_BROADCAST_0 + (ADDRESS_ULTRASONIC_BROADCAST_STEP * i)));
        }

        try {
            _mux.enableBroadcastMode();
        } catch (InterruptedException | LynxNackException e) {
            e.printStackTrace();
        }
    }

    public boolean pokeMuxAggressively() {
        try {
            _mux.enableBroadcastMode();
            return true;
        } catch (InterruptedException | LynxNackException e) {
            e.printStackTrace();
            return false;
        }
    }

    public void reset() throws InterruptedException {
        _mux.startReset();
        Thread.sleep(100);
        _mux.stopReset();
    }

    public byte readFromSensor(int port, int register) throws LynxNackException, InterruptedException {
        if (_broadcastMode) {
            return _ultrasonicBroadcastSensors[port].read8(register);
        }

        if (_mux.getActivePort() != port) {
            _mux.setActivePort(port);
        }

        return _ultrasonicSensor.read8(register);
    }

    public double cmOptical(int port) throws InterruptedException, LynxNackException {
        return (double) (readFromSensor(port, REGISTER_OPTICAL_READING) & 0xFF);
    }

    public double cmUltrasonic(int port) throws InterruptedException, LynxNackException {
        return (double) (readFromSensor(port, REGISTER_ULTRASONIC_READING) & 0xFF);
    }
}
