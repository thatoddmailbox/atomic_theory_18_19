package org.firstinspires.ftc.teamcode.opmodes.tests;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.opmodes.auto.AutoAligner;
import org.firstinspires.ftc.teamcode.robot.Robot;

@Autonomous(name="Distance test", group="Tests")
public class CenterCornerTest extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {
        waitForStart();

        Robot robot = new Robot(this, false);
        AutoAligner aligner = new AutoAligner();

        aligner.cornerCenterRobot(robot);
//        aligner.pidCornerCenterRobot(robot);

        while (opModeIsActive()) {
            telemetry.addData("range left", robot.rangeLeft.cmUltrasonic() * 10);
            telemetry.addData("range right", robot.rangeRight.cmUltrasonic() * 10);
            telemetry.addData("distance diff", robot.rangeLeft.cmUltrasonic() * 10 - robot.rangeLeft.cmUltrasonic() * 10);

            telemetry.update();
            idle();
        }
    }
}