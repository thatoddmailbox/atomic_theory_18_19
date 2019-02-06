package org.firstinspires.ftc.teamcode.opmodes.auto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.utils.StartingPosition;

@Autonomous(name="Auto - crater, safe")
public class AutoCraterSafe extends AutoMain {
    @Override
    public StartingPosition getStartingPosition() {
        return StartingPosition.CRATER;
    }

    @Override
    public boolean isSafeAuto() {
        return true;
    }
}