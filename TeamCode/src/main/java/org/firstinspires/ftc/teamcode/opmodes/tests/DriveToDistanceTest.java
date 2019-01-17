package org.firstinspires.ftc.teamcode.opmodes.tests;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.opmodes.auto.AutoAligner;
import org.firstinspires.ftc.teamcode.robot.Robot;

@Autonomous(name="Drive to distance test", group="Tests")
public class DriveToDistanceTest extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {
        waitForStart();

        Robot robot = new Robot(this, false);

        AutoAligner autoAligner = new AutoAligner(this);
        autoAligner.driveToDistance(robot, Robot.Direction.FORWARD, 50, 2, true);
    }
}
