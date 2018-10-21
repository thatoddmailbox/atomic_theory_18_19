package org.firstinspires.ftc.teamcode.opmodes.auto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.utils.Alliance;
import org.firstinspires.ftc.teamcode.utils.StartingPosition;

@Autonomous(name="Auto - red, depot")
public class AutoRedDepot extends AutoMain {
    @Override
    public Alliance getCurrentAlliance() {
        return Alliance.RED;
    }

    @Override
    public StartingPosition getStartingPosition() {
        return StartingPosition.DEPOT;
    }
}
