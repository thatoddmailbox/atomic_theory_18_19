package org.firstinspires.ftc.teamcode.opmodes.tests;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.blackbox.MatchPhase;
import org.firstinspires.ftc.teamcode.robot.Robot;
import org.firstinspires.ftc.teamcode.utils.RobotFeature;

@Autonomous(name="Super turning test", group="Tests")
public class SuperTurningTest extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        try (Robot robot = new Robot(MatchPhase.TEST, this, new RobotFeature[] {
                RobotFeature.IMU
        })) {
            robot.resetHeading();

            waitForStart();
            robot.handleMatchStart();

            ElapsedTime timer = new ElapsedTime();
            timer.reset();
            robot.turn(90);
            telemetry.addData("Turn length", timer.milliseconds() / 1000);
            telemetry.update();

            sleep(2000);
        }
    }
}
