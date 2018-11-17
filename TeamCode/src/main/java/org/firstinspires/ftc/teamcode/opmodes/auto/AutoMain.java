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

        robot.teamMarker.setPosition(Robot.SERVO_TEAM_MARKER_HELD);

        waitForStart();

        telemetry.addData("Status", "Running");
        telemetry.update();

        // unlatch
        robot.latch.setPower(-1);
        sleep(7200);
        robot.latch.setPower(0);

        sleep(100);

        telemetry.addData("Heading - unlatch", robot.getHeading());
        telemetry.update();

        // strafe away from lander
        if (getStartingPosition() == StartingPosition.CRATER) {
            robot.driveMotors(0.4, -0.4, -0.4, 0.4);
            sleep(200);
            robot.driveMotors(0, 0, 0, 0);
        } else {
            robot.driveMotors(1, -1, -1, 1);
            sleep(200);
            robot.driveMotors(0, 0, 0, 0);
        }

        // move forward to have all minerals in view
        robot.driveMotors(0.4, 0.4, 0.4, 0.4);
        sleep((getStartingPosition() == StartingPosition.CRATER ? 100 : 100));
        robot.driveMotors(0, 0, 0, 0);

        // detect mineral
        robot.activateTfod();
        sleep(2000); // TODO: how long of a delay is needed? is any?

        MineralPosition goldMineral = robot.findGoldMineralDifferent();
        telemetry.addData("Gold mineral position", goldMineral.toString());
        telemetry.addData("Heading - mineral", robot.getHeading());
        telemetry.update();

        // come out from lander
        robot.driveMotors(0.4, -0.4, -0.4, 0.4);
        sleep(1000);
        robot.driveMotors(0, 0, 0, 0);

        // Move left/right depending on mineral position
        if (goldMineral == MineralPosition.LEFT) {
            robot.driveMotors(0.5, 0.5, 0.5, 0.5);
            sleep(500);
            robot.driveMotors(0, 0, 0, 0);
        } else if (goldMineral == MineralPosition.RIGHT) {
            robot.driveMotors(-0.5, -0.5, -0.5, -0.5);
            sleep(500);
            robot.driveMotors(0, 0, 0, 0);
        }

        // if at crater, try to go in
        if (getStartingPosition() == StartingPosition.CRATER) {
            // turn 90
            robot.lessBadTurn(90);

            // attack
            robot.driveMotors(-0.8, -0.8, -0.8, -0.8);
            sleep(700);
            robot.driveMotors(0, 0, 0, 0);
        } else {
            // turn 90
            robot.lessBadTurn(90);

            // Push mineral
            robot.driveMotors(-0.5, -0.5, -0.5, -0.5);
            sleep(1200);
            robot.driveMotors(0, 0, 0, 0);

            robot.lessBadTurn(0);

            if (goldMineral == MineralPosition.LEFT) {
                robot.driveMotors(-0.5, -0.5, -0.5, -0.5);
                sleep(500);
                robot.driveMotors(0, 0, 0, 0);
            } else if (goldMineral == MineralPosition.RIGHT) {
                robot.driveMotors(0.5, 0.5, 0.5, 0.5);
                sleep(500);
                robot.driveMotors(0, 0, 0, 0);
            }

            // Drop team marker
            if (getStartingPosition() == StartingPosition.DEPOT) {
                robot.teamMarker.setPosition(Robot.SERVO_TEAM_MARKER_DEPOSIT);
                sleep(1000);
            }

            robot.lessBadTurn(45);

            robot.driveMotors(-1, -1, -1, -1);
            sleep(3000);
            robot.driveMotors(0, 0, 0, 0);

//            robot.lessBadTurn(270);
        }

        robot.deactivateTfod();
        robot.teamMarker.setPosition(Robot.SERVO_TEAM_MARKER_HELD);

        while (opModeIsActive()) {
            telemetry.update();
            idle();
        }
    }
}
