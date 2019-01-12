package org.firstinspires.ftc.teamcode.opmodes.tests;

import com.qualcomm.hardware.rev.Rev2mDistanceSensor;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.opmodes.auto.AutoAligner;
import org.firstinspires.ftc.teamcode.robot.Robot;

@Autonomous(name="Wall alignment + distance test", group="Tests")
public class WallAlignmentAtDistanceTest extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {
        waitForStart();

        Robot robot = new Robot(this, false);
        AutoAligner aligner = new AutoAligner();

        while (opModeIsActive()) {
            telemetry.addData("range left", robot.rangeLeft.cmUltrasonic() * 10);
            telemetry.addData("range right", robot.rangeRight.cmUltrasonic() * 10);

            telemetry.update();

            aligner.driveAlignDistanceRobot(robot, 0.85, 300);
            idle();
        }
    }
}