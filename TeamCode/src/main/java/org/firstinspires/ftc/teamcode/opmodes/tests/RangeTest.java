package org.firstinspires.ftc.teamcode.opmodes.tests;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.opmodes.auto.AutoAligner;
import org.firstinspires.ftc.teamcode.robot.Robot;

@Autonomous(name="Range test", group="Tests")
public class RangeTest extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {
        waitForStart();

        Robot robot = new Robot(this, false);

        while (opModeIsActive()) {
            telemetry.addData("range left optical (cm)", robot.rangeLeft.cmOptical());
            telemetry.addData("range left ultrasonic (cm)", robot.rangeLeft.cmUltrasonic());

            telemetry.addData("range right optical (cm)", robot.rangeRight.cmOptical());
            telemetry.addData("range right ultrasonic (cm)", robot.rangeRight.cmUltrasonic());

            telemetry.update();

            idle();
        }
    }
}
