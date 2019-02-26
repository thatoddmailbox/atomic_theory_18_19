package org.firstinspires.ftc.teamcode.opmodes.tests;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.blackbox.MatchPhase;
import org.firstinspires.ftc.teamcode.robot.Robot;
import org.firstinspires.ftc.teamcode.utils.RobotFeature;

@Autonomous(name="Center corner test", group="Tests")
public class CenterCornerTest extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {
        try (Robot robot = new Robot(MatchPhase.TEST, this, new RobotFeature[] {
                RobotFeature.IMU
        })) {
            waitForStart();
            robot.handleMatchStart();

            robot.aligner.centerInCorner(2, true);
        }
    }
}