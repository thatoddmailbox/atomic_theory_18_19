package org.firstinspires.ftc.teamcode.opmodes.tests;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.blackbox.MatchPhase;
import org.firstinspires.ftc.teamcode.opmodes.auto.AutoAligner;
import org.firstinspires.ftc.teamcode.robot.Robot;
import org.firstinspires.ftc.teamcode.utils.Direction;
import org.firstinspires.ftc.teamcode.utils.RobotFeature;

@Autonomous(name="Drive to distance test", group="Tests")
public class DriveToDistanceTest extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {
        try (Robot robot = new Robot(MatchPhase.TEST, this, new RobotFeature[] {
                RobotFeature.IMU
        })) {
            waitForStart();
            robot.handleMatchStart();

            robot.aligner.driveToDistance(Direction.FORWARD, Direction.RIGHT, true, 300, 5, true);
        }
    }
}
