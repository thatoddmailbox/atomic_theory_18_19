package org.firstinspires.ftc.teamcode.opmodes.auto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.utils.StartingPosition;

@Autonomous(name="Auto - crater")
public class AutoCrater extends AutoMain {
    @Override
    public StartingPosition getStartingPosition() {
        return StartingPosition.CRATER;
    }
}
