package org.firstinspires.ftc.teamcode.opmodes.tests;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.blackbox.MatchPhase;
import org.firstinspires.ftc.teamcode.robot.Robot;
import org.firstinspires.ftc.teamcode.utils.RobotFeature;

@Autonomous(name="Range test", group="Tests")
public class RangeTest extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {
        try (Robot robot = new Robot(MatchPhase.TEST, this, new RobotFeature[] {})) {
            waitForStart();
            robot.handleMatchStart();

            long lastTime = System.currentTimeMillis();
            while (opModeIsActive()) {
                long diff = (System.currentTimeMillis() - lastTime);
                telemetry.addData("Loop time", diff);
                lastTime = System.currentTimeMillis();
                robot.logSensors();
            }
        }
    }
}
