package org.firstinspires.ftc.teamcode.opmodes.auto;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.teamcode.robot.Robot;
import org.firstinspires.ftc.teamcode.utils.MineralPosition;
import org.firstinspires.ftc.teamcode.utils.PersistentHeading;
import org.firstinspires.ftc.teamcode.utils.PowerSetting;
import org.firstinspires.ftc.teamcode.utils.StartingPosition;

public abstract class AutoMain extends LinearOpMode {

    public abstract StartingPosition getStartingPosition();

    @Override
    public void runOpMode() throws InterruptedException {
        telemetry.addData("Status", "Starting...");
        telemetry.update();

        PersistentHeading.clearSavedHeading();

        Robot robot = new Robot(this, true);

        telemetry.addData("Status", "Ready to go");
        telemetry.addData("Starting position", getStartingPosition());
        telemetry.update();

        robot.teamMarker.setPosition(Robot.SERVO_TEAM_MARKER_HELD);
        robot.resetHeading();

        waitForStart();

        telemetry.addData("Status", "Running");
        telemetry.update();

        // unlatch TODO: make this an encoder value
        robot.latch.setPower(-1);
        sleep(7200);
        robot.latch.setPower(0);

        sleep(100);

        telemetry.addData("Heading - unlatch", robot.getHeading());
        telemetry.update();

        // turn to realign
        robot.lessBadTurn(0);

        // strafe away from lander //TODO: These two shouldn't be different times + ENCODE
        if (getStartingPosition() == StartingPosition.CRATER) {
            robot.driveMotors(0.4, -0.4, -0.4, 0.4);
            sleep(200);
            robot.driveMotors(0, 0, 0, 0);
        } else {
            robot.driveMotors(1, -1, -1, 1);
            sleep(200);
            robot.driveMotors(0, 0, 0, 0);
        }

        // turn to realign
        robot.lessBadTurn(0);

        // move forward to have all minerals in view TODO: ENCODE ME
        robot.driveMotors(0.4, 0.4, 0.4, 0.4);
        sleep((getStartingPosition() == StartingPosition.CRATER ? 100 : 100));
        robot.driveMotors(0, 0, 0, 0);

        // turn to realign
        robot.lessBadTurn(0);

        // detect mineral
        robot.activateTfod();
        sleep(2000); // TODO: how long of a delay is needed? is any?

        MineralPosition goldMineral = robot.findGoldMineralDifferent();
        telemetry.addData("Gold mineral position", goldMineral.toString());
        telemetry.addData("Heading - mineral", robot.getHeading());
        telemetry.update();

        // come out from lander TODO: ENCODE ME
        robot.driveMotors(0.4, -0.4, -0.4, 0.4);
        sleep(1200);
        robot.driveMotors(0, 0, 0, 0);

        // turn to realign
        robot.lessBadTurn(0);

        // Move left/right depending on mineral position TODO: ENCODE ME
        if (goldMineral == MineralPosition.LEFT) {
            robot.driveMotors(0.5, 0.5, 0.5, 0.5);
            sleep(800);
            robot.driveMotors(0, 0, 0, 0);
        } else if (goldMineral == MineralPosition.RIGHT) {
            robot.driveMotors(-0.5, -0.5, -0.5, -0.5);
            sleep(800);
            robot.driveMotors(0, 0, 0, 0);
        }

        // if at crater, try to go in
        if (getStartingPosition() == StartingPosition.CRATER) {
            // turn 90
            robot.lessBadTurn(90); //TODO: New plate, don't need turn

            // hit mineral
            robot.driveMotors(-0.8, -0.8, -0.8, -0.8);
            sleep(300);
            // back up a little
            robot.driveMotors(0.8, 0.8, 0.8, 0.8);
            sleep(200);
            robot.driveMotors(0, 0, 0, 0);

            // go back to OG position
            if (goldMineral == MineralPosition.LEFT) {
                robot.driveMotors(-0.5, 0.5, 0.5, -0.5);
                sleep(800);
                robot.driveMotors(0, 0, 0, 0);
            } else if (goldMineral == MineralPosition.RIGHT) {
                robot.driveMotors(0.5, -0.5, -0.5, 0.5);
                sleep(800);
                robot.driveMotors(0, 0, 0, 0);
            }

            // Head towards depot diagonally
            robot.driveMotors(0.8, -0.8, -0.8, 0.8);
            sleep(1500);
            robot.driveMotors(0, 0, 0, 0);

            // Rotate right by 45 degrees so we're pointed straight
            robot.lessBadTurn(-45);

            long timeToDepot = 1200;
            if (false) {
                AutoAligner aligner = new AutoAligner();
                boolean notThereYet = true;
                ElapsedTime elapsedTime = new ElapsedTime();
                while (opModeIsActive() && elapsedTime.milliseconds() < timeToDepot+100) {
                    aligner.driveAlignDistanceRobot(robot, 0.8, 20);
                    idle();
                }
            } else {
                // Drive forwards into depot
                robot.driveMotors(0.8, 0.8, 0.8, 0.8);
                sleep(timeToDepot);
                robot.driveMotors(0, 0, 0, 0);
            }
        } else {
            // turn 90
            robot.lessBadTurn(90); //TODO: New plate, don't need turn

            // Push mineral
            robot.driveMotors(-0.5, -0.5, -0.5, -0.5);
            sleep(1400);
            robot.driveMotors(0, 0, 0, 0);

            robot.lessBadTurn(0);

            if (goldMineral == MineralPosition.LEFT) {
                robot.driveMotors(-0.5, -0.5, -0.5, -0.5);
                sleep(800);
                robot.driveMotors(0, 0, 0, 0);
            } else if (goldMineral == MineralPosition.RIGHT) {
                robot.driveMotors(0.5, 0.5, 0.5, 0.5);
                sleep(800);
                robot.driveMotors(0, 0, 0, 0);
            }
        }

        // Drop team marker
        robot.teamMarker.setPosition(Robot.SERVO_TEAM_MARKER_DEPOSIT);
        sleep(1000);
        robot.lessBadTurn(getStartingPosition() == StartingPosition.CRATER ? 45 : -45);

        robot.driveMotors(0.5, -0.5, -0.5, 0.5);
        sleep(250);
        robot.driveMotors(0, 0, 0, 0);

        robot.driveMotors(-0.8, -0.8, -0.8, -0.8);
        sleep(3000);
        robot.driveMotors(0, 0, 0, 0);

        robot.deactivateTfod();
        robot.teamMarker.setPosition(Robot.SERVO_TEAM_MARKER_HELD);

        // lower latch
        robot.latch.setPower(1);
        ElapsedTime timer = new ElapsedTime();

        while (opModeIsActive() && timer.seconds() < 6) {
            telemetry.addData("Time", timer.seconds());
            telemetry.update();
            idle();
        }

        robot.latch.setPower(0);

        // save heading
        PersistentHeading.saveHeading(robot.getHeading());
    }
}
