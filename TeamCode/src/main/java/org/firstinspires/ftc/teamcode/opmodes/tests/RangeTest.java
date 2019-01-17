package org.firstinspires.ftc.teamcode.opmodes.tests;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.robot.Robot;

@Autonomous(name="Range test", group="Tests")
public class RangeTest extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {
        waitForStart();

        Robot robot = new Robot(this, false);

        while (opModeIsActive()) {
            robot.logSensors();
        }
    }
}
