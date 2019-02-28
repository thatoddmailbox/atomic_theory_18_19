package org.firstinspires.ftc.teamcode.hardware;

import com.qualcomm.hardware.lynx.LynxNackException;

public class MuxedRangeSensor {
    private UltrasonicHub _hub;
    private int _port;

    public MuxedRangeSensor(UltrasonicHub hub, int port) {
        _hub = hub;
        _port = port;
    }

    public double cmOptical() throws LynxNackException, InterruptedException {
        return _hub.cmOptical(_port);
    }

    public double cmUltrasonic() throws LynxNackException, InterruptedException {
        return _hub.cmUltrasonic(_port);
    }
}
