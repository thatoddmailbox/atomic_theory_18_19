package org.firstinspires.ftc.teamcode.hardware;

import com.qualcomm.hardware.lynx.LynxNackException;

public class MuxedRangeSensor {
    private UltrasonicHub _hub;
    private int _port;

    private long _lastPoke;

    public MuxedRangeSensor(UltrasonicHub hub, int port) {
        _hub = hub;
        _port = port;
        _lastPoke = System.currentTimeMillis();
    }

    public double cmOptical() throws LynxNackException, InterruptedException {
        return _hub.cmOptical(_port);
    }

    public double cmUltrasonic() throws LynxNackException, InterruptedException {
        double reading = _hub.cmUltrasonic(_port);
        if (reading == 0 && (System.currentTimeMillis() - _lastPoke) > 10) {
            _hub.pokeMuxAggressively();
            _lastPoke = System.currentTimeMillis();
        }
        return reading;
    }
}
