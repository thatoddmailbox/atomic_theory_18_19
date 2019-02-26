package org.firstinspires.ftc.teamcode.opmodes.tests;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.exception.RobotCoreException;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.PIDCoefficients;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.blackbox.MatchPhase;
import org.firstinspires.ftc.teamcode.robot.Robot;
import org.firstinspires.ftc.teamcode.utils.PIDController;
import org.firstinspires.ftc.teamcode.utils.PIDLogger;
import org.firstinspires.ftc.teamcode.utils.RobotFeature;

import java.net.SocketException;
import java.net.UnknownHostException;

@Autonomous(name="Latch test", group="Tests")
public class UnlatchTest extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        telemetry.addData("Status", "Starting...");
        telemetry.update();

        try (Robot robot = new Robot(MatchPhase.TEST, this, new RobotFeature[] {})) {
            telemetry.addData("Status", "Ready to go");
            telemetry.update();

            waitForStart();
            robot.handleMatchStart();

            int latchLeftStart = robot.latchLeft.getCurrentPosition();
            int latchRightStart = robot.latchRight.getCurrentPosition();

            while (opModeIsActive()) {
                if (gamepad2.y) {
                    robot.latchLeft.setPower(-0.8);
                    robot.latchRight.setPower(-1.0);
                    robot.latchRight.setTargetPosition(robot.latchRight.getCurrentPosition() - 100);
                    robot.latchLeft.setTargetPosition(robot.latchLeft.getCurrentPosition() - 100);

                } else if (gamepad2.a) {
                    robot.latchLeft.setPower(0.8);
                    robot.latchRight.setPower(1.0);
                    robot.latchLeft.setTargetPosition(robot.latchLeft.getCurrentPosition() + 100);
                    robot.latchRight.setTargetPosition(robot.latchRight.getCurrentPosition() + 100);

                } else if (gamepad2.dpad_left) {
                    robot.latchLeft.setPower(-0.8);
                    robot.latchLeft.setTargetPosition(robot.latchLeft.getCurrentPosition() - 15);

                } else if (gamepad2.dpad_right) {
                    robot.latchLeft.setPower(+0.8);
                    robot.latchLeft.setTargetPosition(robot.latchLeft.getCurrentPosition() + 15);

                } else {
                    robot.latchLeft.setPower(0);
                    robot.latchRight.setPower(0);
                }
                idle();
            }

            robot.latchLeft.setPower(0);
            robot.latchRight.setPower(0);
        }
    }
}
