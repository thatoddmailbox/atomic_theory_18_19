package org.firstinspires.ftc.teamcode.opmodes.tests;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.exception.RobotCoreException;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.PIDCoefficients;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.robot.Robot;
import org.firstinspires.ftc.teamcode.utils.PIDController;
import org.firstinspires.ftc.teamcode.utils.PIDLogger;

import java.net.SocketException;
import java.net.UnknownHostException;

@Autonomous(name="Unlatch test", group="Tests")
public class UnlatchTest extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        telemetry.addData("Status", "Starting...");
        telemetry.update();

        Robot robot = new Robot(this, false);

        telemetry.addData("Status", "Ready to go");
        telemetry.update();

        waitForStart();

        robot.latch.setPower(1);
        ElapsedTime timer = new ElapsedTime();

        while (opModeIsActive() && timer.seconds() < 6) {
            telemetry.addData("Time", timer.seconds());
            telemetry.update();
            idle();
        }

        robot.latch.setPower(0);
    }
}
