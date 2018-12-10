package org.firstinspires.ftc.teamcode.opmodes.auto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.utils.StartingPosition;

@Autonomous(name="Auto - depot")
public class AutoDepot extends AutoMain {
    @Override
    public StartingPosition getStartingPosition() {
        return StartingPosition.DEPOT;
    }
}
