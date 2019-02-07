package org.firstinspires.ftc.teamcode.opmodes.tests;

import android.net.wifi.p2p.WifiP2pManager;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.internal.network.WifiDirectAgent;
import org.firstinspires.ftc.teamcode.robot.Robot;
import org.firstinspires.ftc.teamcode.utils.MineralPosition;

@Autonomous(name="Camera test", group="Tests")
public class CameraTest extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {
        waitForStart();

        Robot robot = new Robot(this, true);

        robot.activateTfod();

        while (opModeIsActive()) {
            sleep(100);
            MineralPosition goldMineral = robot.findGoldMineralDifferent();
            telemetry.addData("gold mineral", goldMineral.name());
            telemetry.update();
        }

        robot.deactivateTfod();
    }
}
