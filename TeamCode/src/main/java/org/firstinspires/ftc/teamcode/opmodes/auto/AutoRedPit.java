package org.firstinspires.ftc.teamcode.opmodes.auto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.utils.Alliance;
import org.firstinspires.ftc.teamcode.utils.StartingPosition;

@Autonomous(name="Auto - red, pit")
public class AutoRedPit extends AutoMain {
    @Override
    public Alliance getCurrentAlliance() {
        return Alliance.RED;
    }

    @Override
    public StartingPosition getStartingPosition() {
        return StartingPosition.PIT;
    }
}
