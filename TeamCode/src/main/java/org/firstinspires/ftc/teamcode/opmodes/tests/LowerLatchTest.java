package org.firstinspires.ftc.teamcode.opmodes.tests;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.blackbox.MatchPhase;
import org.firstinspires.ftc.teamcode.robot.Robot;
import org.firstinspires.ftc.teamcode.utils.RobotFeature;

@Autonomous(name="Lower latch test")
public class LowerLatchTest extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        try (Robot robot = new Robot(MatchPhase.TEST, this, new RobotFeature[] {})) {
            waitForStart();
            robot.handleMatchStart();

            int latchLeftStart = robot.latchLeft.getCurrentPosition();
            int latchRightStart = robot.latchRight.getCurrentPosition();

            robot.latchLeft.setPower(0.8);
            robot.latchLeft.setTargetPosition(latchLeftStart + Robot.LATCH_DISTANCE);
            robot.latchRight.setPower(1.0);
            robot.latchRight.setTargetPosition(latchRightStart + Robot.LATCH_DISTANCE);

            while (robot.latchLeft.isBusy() || robot.latchRight.isBusy()) {
                idle();
            }

            robot.latchLeft.setPower(0.0);
            robot.latchRight.setPower(0.0);
        }
    }
}
