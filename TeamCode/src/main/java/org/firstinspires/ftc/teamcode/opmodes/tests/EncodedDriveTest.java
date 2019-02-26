package org.firstinspires.ftc.teamcode.opmodes.tests;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.blackbox.MatchPhase;
import org.firstinspires.ftc.teamcode.robot.Robot;
import org.firstinspires.ftc.teamcode.utils.RobotFeature;

@Autonomous(name="Encoded drive test", group="Tests")
public class EncodedDriveTest extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {
        try (Robot robot = new Robot(MatchPhase.TEST, this, new RobotFeature[] {
                RobotFeature.IMU
        })) {
            waitForStart();
            robot.handleMatchStart();

            robot.driveTicks(1000, 0.9, 0.9, 0.9, 0.9);

            sleep(20000);
        }
    }
}
