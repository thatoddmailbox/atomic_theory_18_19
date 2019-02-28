package org.firstinspires.ftc.teamcode.hardware;

import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.hardware.lynx.LynxNackException;
import com.qualcomm.hardware.lynx.commands.core.LynxI2cWriteSingleByteCommand;
import com.qualcomm.hardware.lynx.commands.standard.LynxAck;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.I2cAddr;

/*
 * TCA9545A I2C multiplexer
 * http://www.ti.com/lit/ds/symlink/tca9545a.pdf
 */
public class TCA9545A {
    private LynxModule _hub;
    private int _hubPort;
    private I2cAddr _address;

    private int _activePort;

    private DigitalChannel _resetLine;

    public TCA9545A(LynxModule hub, int hubPort, DigitalChannel resetLine, boolean a1, boolean a0) {
        _hub = hub;
        _hubPort = hubPort;
        _address = I2cAddr.create7bit(0b1110000 | ((a1 ? 1 : 0) << 1) | (a0 ? 1 : 0));
        _resetLine = resetLine;
    }

    public void startReset() {
        _resetLine.setState(false);
        _activePort = -1;
    }

    public void stopReset() {
        _resetLine.setState(true);
        _activePort = -1;
    }

    public void reset() throws InterruptedException {
        startReset();
        Thread.sleep(100);
        stopReset();
    }

    public int getActivePort() {
        return _activePort;
    }

    public void setActivePort(int port) throws InterruptedException, LynxNackException {
        if (port < 0 || port > 3) {
            throw new RuntimeException("Port for TCA9545A out of range!");
        }

        byte controlByte = (byte) (1 << port);

        sendControlByte(controlByte);

        _activePort = port;
    }

    public void enableBroadcastMode() throws InterruptedException, LynxNackException {
        sendControlByte((byte) 0b00001111);
    }

    public void sendControlByte(byte controlByte) throws InterruptedException, LynxNackException {
        // is directly writing these i2c commands, bypassing the i2c lock and existing transaction stuff dangerous? yes
        // is it fast? probably
        // should this be replaced by the more sane usage of a LynxI2CDeviceSynch? maybe
        // will this mysteriously blow up during a competition? most likely
        // unfortunately, LynxI2CDeviceSynch assumes that i2c devices have registers and the tca9545 does not
        // it could be hacked around probably (make the register 0x00 and then the "data" would be the control byte)
        // but that's annoying and slower so hopefully this works :~)
        LynxI2cWriteSingleByteCommand command = new LynxI2cWriteSingleByteCommand(_hub, _hubPort, _address, controlByte);
        command.sendReceive();
    }
}
