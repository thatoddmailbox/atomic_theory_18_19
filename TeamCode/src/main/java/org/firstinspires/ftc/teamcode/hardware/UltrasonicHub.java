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
    private static final byte REGISTER_ULTRASONIC_READING = 0x04;

    private LynxModule _hub;
    private int _hubPort;

    private I2cAddr _ultrasonicAddress;

    private TCA9545A _mux;
    private LynxI2cDeviceSynch _ultrasonicSensor;

    public UltrasonicHub(Context context, LynxModule hub, int hubPort, DigitalChannel muxReset) {
        if (!hub.isCommandSupported(LynxI2cWriteReadMultipleBytesCommand.class)) {
            throw new RuntimeException("Expansion hub firmware requires update! LynxI2cWriteReadMultipleBytesCommand not supported!");
        }

        _hub = hub;
        _hubPort = hubPort;
        _ultrasonicAddress = I2cAddr.create8bit(0x28);

        _mux = new TCA9545A(_hub, _hubPort, muxReset, false, false);
        _ultrasonicSensor = new LynxI2cDeviceSynchV2(context, _hub, _hubPort);
        _ultrasonicSensor.setI2cAddr(_ultrasonicAddress);
    }

    public void reset() throws InterruptedException {
        _mux.startReset();
        Thread.sleep(100);
        _mux.stopReset();
    }

    public byte getReadingFromSensor(int port) throws LynxNackException, InterruptedException {
        if (_mux.getActivePort() != port) {
            _mux.setActivePort(port);
        }

        return _ultrasonicSensor.read8(REGISTER_ULTRASONIC_READING);
    }
}
