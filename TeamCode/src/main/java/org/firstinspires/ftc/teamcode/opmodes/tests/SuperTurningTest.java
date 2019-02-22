package org.firstinspires.ftc.teamcode.opmodes.tests;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.blackbox.MatchPhase;
import org.firstinspires.ftc.teamcode.robot.Robot;

@Autonomous(name="Super turning test", group="Tests")
public class SuperTurningTest extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        Robot robot = new Robot(MatchPhase.TEST, this, false);

        robot.resetHeading();

        waitForStart();

        ElapsedTime timer = new ElapsedTime();
        timer.reset();
        robot.turn(90);
        telemetry.addData("Turn length", timer.milliseconds() / 1000);
        telemetry.update();

        sleep(2000);
    }
}
