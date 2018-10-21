package org.firstinspires.ftc.teamcode.opmodes.auto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.utils.Alliance;
import org.firstinspires.ftc.teamcode.utils.StartingPosition;

@Autonomous(name="Auto - blue, depot")
public class AutoBlueDepot extends AutoMain {
    @Override
    public Alliance getCurrentAlliance() {
        return Alliance.BLUE;
    }

    @Override
    public StartingPosition getStartingPosition() {
        return StartingPosition.DEPOT;
    }
}
