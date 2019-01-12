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

@Autonomous(name="Super turning test", group="Tests")
public class SuperTurningTest extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        Robot robot = new Robot(this, false);

        robot.resetHeading();

        waitForStart();

        ElapsedTime timer = new ElapsedTime();
        timer.reset();
        robot.lessBadTurn(90);
        telemetry.addData("Turn length", timer.milliseconds() / 1000);
        telemetry.update();

        sleep(2000);
    }
}
