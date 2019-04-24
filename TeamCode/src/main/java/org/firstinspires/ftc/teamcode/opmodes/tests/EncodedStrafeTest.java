package org.firstinspires.ftc.teamcode.opmodes.tests;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.blackbox.MatchPhase;
import org.firstinspires.ftc.teamcode.robot.Robot;
import org.firstinspires.ftc.teamcode.utils.RobotFeature;

@Autonomous(name="Encoded strafe test", group="Tests")
public class EncodedStrafeTest extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {
        try (Robot robot = new Robot(MatchPhase.TEST, this, new RobotFeature[] {
                RobotFeature.IMU
        })) {
            waitForStart();
            robot.handleMatchStart();

//            robot.driveTicks(900, 1, -1, -1, 1);
            robot.strafeTicks(900, 1, -0.6, -1, 0.6);
            sleep(500);
            robot.driveMotors(1, -0.6, -1, 0.6);
            sleep(900);
            robot.driveMotors(0, 0, 0, 0);

            sleep(20000);
        }
    }
}
