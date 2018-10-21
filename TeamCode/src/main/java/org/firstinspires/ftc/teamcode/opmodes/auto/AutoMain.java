package org.firstinspires.ftc.teamcode.opmodes.auto;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.robot.Robot;
import org.firstinspires.ftc.teamcode.utils.Alliance;
import org.firstinspires.ftc.teamcode.utils.StartingPosition;

public abstract class AutoMain extends LinearOpMode {

    public abstract Alliance getCurrentAlliance();
    public abstract StartingPosition getStartingPosition();

    @Override
    public void runOpMode() {
        Robot robot = new Robot(hardwareMap);

        telemetry.addData("Status", "Ready to go");
        telemetry.addData("Alliance", getCurrentAlliance());
        telemetry.addData("Starting position", getStartingPosition());
        telemetry.update();

        waitForStart();

        telemetry.addData("Status", "Running");
        telemetry.update();

        while (opModeIsActive()) {
            idle();
        }
    }
}
