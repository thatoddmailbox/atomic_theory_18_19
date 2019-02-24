package org.firstinspires.ftc.teamcode.utils;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.blackbox.sensors.WrappedLynxModule;

public class LynxPumperRunnable implements Runnable {
    private LinearOpMode _opMode;
    private WrappedLynxModule[] _hubs;

    public LynxPumperRunnable(LinearOpMode opMode, WrappedLynxModule[] hubs) {
        _opMode = opMode;
        _hubs = hubs;
    }

    @Override
    public void run() {
        while (!_opMode.isStopRequested()) {
            for (WrappedLynxModule module : _hubs) {
                module.pumpDatastream();
            }
            try {
                Thread.sleep(10);
            } catch (InterruptedException e) {
                // do nothing, we're probably about to die anyways
            }
        }
    }
}
