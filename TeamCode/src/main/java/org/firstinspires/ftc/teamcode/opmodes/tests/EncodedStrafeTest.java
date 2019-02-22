package org.firstinspires.ftc.teamcode.opmodes.tests;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.blackbox.MatchPhase;
import org.firstinspires.ftc.teamcode.robot.Robot;

@Autonomous(name="Encoded strafe test", group="Tests")
public class EncodedStrafeTest extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {
        waitForStart();

        Robot robot = new Robot(MatchPhase.TEST, this, false);

        robot.driveTicks(1000, 0.9, -0.9, -0.9, 0.9);
        
        sleep(20000);
    }
}
