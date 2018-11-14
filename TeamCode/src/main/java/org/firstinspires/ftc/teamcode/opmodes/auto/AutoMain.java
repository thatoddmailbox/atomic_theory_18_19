package org.firstinspires.ftc.teamcode.opmodes.auto;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.teamcode.robot.Robot;
import org.firstinspires.ftc.teamcode.utils.Alliance;
import org.firstinspires.ftc.teamcode.utils.MineralPosition;
import org.firstinspires.ftc.teamcode.utils.PowerSetting;
import org.firstinspires.ftc.teamcode.utils.StartingPosition;

public abstract class AutoMain extends LinearOpMode {

    public abstract Alliance getCurrentAlliance();
    public abstract StartingPosition getStartingPosition();

    @Override
    public void runOpMode() throws InterruptedException {
        telemetry.addData("Status", "Starting...");
        telemetry.update();

        Robot robot = new Robot(this, true);

        telemetry.addData("Status", "Ready to go");
        telemetry.addData("Alliance", getCurrentAlliance());
        telemetry.addData("Starting position", getStartingPosition());
        telemetry.update();

        waitForStart();

        telemetry.addData("Status", "Running");
        telemetry.update();

        // unlatch
        robot.latch.setPower(-1);
        sleep(7200);
        robot.latch.setPower(0);

        sleep(100);

        // strafe away from lander
        robot.driveMotors(1, -1, -1, 1);
        sleep(200);
        robot.driveMotors(0, 0, 0, 0);

        // move forward to have all minerals in view
        robot.driveMotors(0.4, 0.4, 0.4, 0.4);
        sleep(100);
        robot.driveMotors(0, 0, 0, 0);

        // detect mineral
        robot.activateTfod();
        sleep(2000); // TODO: how long of a delay is needed? is any?

        MineralPosition goldMineral = robot.findGoldMineralDifferent();
        telemetry.addData("Gold mineral position", goldMineral.toString());
        telemetry.update();

        // Move left/right depending on mineral position
        if (goldMineral == MineralPosition.CENTER) {
        } else if (goldMineral == MineralPosition.LEFT) {
            robot.driveMotors(0.5, 0.5, 0.5, 0.5);
            sleep(360);
            robot.driveMotors(0, 0, 0, 0);
        } else if (goldMineral == MineralPosition.RIGHT) {
            robot.driveMotors(-0.5, -0.5, -0.5, -0.5);
            sleep(360);
            robot.driveMotors(0, 0, 0, 0);
        }

        int pushTime = 2000;
        if (getStartingPosition() == StartingPosition.CRATER) {
            pushTime = 1500;
        }

        // Push mineral
        robot.driveMotors(0.6, -0.6, -0.6, 0.6);
        sleep(pushTime);
        robot.driveMotors(0, 0, 0, 0);

        // Go back towards lander if at crater
        if (getStartingPosition() == StartingPosition.CRATER) {
            robot.driveMotors(-0.6, 0.6, 0.6, -0.6);
            sleep(pushTime);
            robot.driveMotors(0, 0, 0, 0);
        }

        // Move back to center
        if (goldMineral == MineralPosition.CENTER) {
        } else if (goldMineral == MineralPosition.LEFT) {
            robot.driveMotors(-0.5, -0.5, -0.5, -0.5);
            sleep(360);
            robot.driveMotors(0, 0, 0, 0);
        } else if (goldMineral == MineralPosition.RIGHT) {
            robot.driveMotors(0.5, 0.5, 0.5, 0.5);
            sleep(360);
            robot.driveMotors(0, 0, 0, 0);
        }

        // Drop team marker
        if (getStartingPosition() == StartingPosition.DEPOT) {
            robot.teamMarker.setPosition(1);
            sleep(500);
            robot.teamMarker.setPosition(0.5);
        }

        // Go left (towards depot/wall) if at crater
        if (getStartingPosition() == StartingPosition.CRATER) {
            robot.driveMotors(-0.7, -0.7, -0.7, -0.7);
            sleep(1500);
            robot.driveMotors(0, 0, 0, 0);
            // Turn left (maybe) 45 degrees to align with wall
            robot.driveMotors(0.5, -0.5, 0.5, -0.5);
            sleep(400);
            robot.driveMotors(0, 0, 0, 0);
        }

        robot.deactivateTfod();

        while (opModeIsActive()) {
            telemetry.update();
            idle();
        }
    }
}
