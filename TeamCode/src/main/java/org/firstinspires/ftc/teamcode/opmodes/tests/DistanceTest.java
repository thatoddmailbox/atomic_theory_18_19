package org.firstinspires.ftc.teamcode.opmodes.tests;

import com.qualcomm.hardware.rev.Rev2mDistanceSensor;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.opmodes.auto.AutoAligner;
import org.firstinspires.ftc.teamcode.robot.Robot;

@Autonomous(name="Distance test", group="Tests")
public class DistanceTest extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {
        waitForStart();

        Robot robot = new Robot(this, false);
        AutoAligner aligner = new AutoAligner();

        while (opModeIsActive()) {
            telemetry.addData("distance left", robot.distanceLeft.getDistance(DistanceUnit.MM));
            telemetry.addData("distance right", robot.distanceRight.getDistance(DistanceUnit.MM));

            telemetry.update();

            aligner.driveAlignDistanceRobot(robot, 0.5, 300);
            idle();
        }
    }
}
