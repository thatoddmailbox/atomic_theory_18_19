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
    public abstract boolean isSafeAuto();

    @Override
    public void runOpMode() throws InterruptedException {
        telemetry.addData("Status", "Starting...");
        telemetry.update();

        PersistentHeading.clearSavedHeading();

        Robot robot = new Robot(this, false);

        telemetry.addData("Status", "Ready to go");
        telemetry.addData("Starting position", getStartingPosition());
        telemetry.update();

        robot.teamMarker.setPosition(Robot.SERVO_TEAM_MARKER_HELD);
        robot.resetHeading();

        waitForStart();

        telemetry.addData("Status", "Running");
        telemetry.update();


        // unlatch TODO: make this an encoder value

        // UNCOMMENT THIS DUMMY
        robot.latchLeft.setPower(-1);
        robot.latchRight.setPower(-1);
        sleep(7200);
        robot.latchLeft.setPower(0);
        robot.latchRight.setPower(0);

        sleep(100);

        telemetry.addData("Heading - unlatch", robot.getHeading());
        telemetry.update();

        // turn to realign
        robot.lessBadTurn(0);

        // strafe away from lander //TODO: These two shouldn't be different times + ENCODE
        robot.driveMotors(1, -1, -1, 1);
        sleep(200);
        robot.driveMotors(0, 0, 0, 0);

        // turn to realign
        robot.lessBadTurn(0, 0.5);

        // NEW AUTO (UNTESTED)
        if (true) {
            AutoAligner aligner = new AutoAligner();

            // Get centered on minerals/lander
            aligner.cornerCenterRobot(robot);

            sleep(1000);

            // Get close to minerals (away from lander)
            aligner.driveToDistance(robot, AutoAligner.Direction.RIGHT, 130);

            sleep(1000);

            // Get in front of cube
            aligner.driveToDistance(robot, AutoAligner.Direction.FORWARD, 87);

            sleep(1000);

            // Hit cube
            aligner.driveToDistance(robot, AutoAligner.Direction.RIGHT, 40);

            sleep(1000);

            robot.lessBadTurn(0, 0.5);

            sleep(1000);

            aligner.cornerCenterRobot(robot);

            return;
        }


        // move forward to have all minerals in view TODO: ENCODE ME
        robot.driveMotors(0.4, 0.4, 0.4, 0.4);
        sleep(100);
        robot.driveMotors(0, 0, 0, 0);

//        // turn to realign
//        robot.lessBadTurn(0);

        // detect mineral
        robot.activateTfod();
        sleep(1000); // TODO: how long of a delay is needed? is any?

        MineralPosition goldMineral = robot.findGoldMineralDifferent();
        telemetry.addData("Gold mineral position", goldMineral.toString());
        telemetry.addData("Heading - mineral", robot.getHeading());
        telemetry.update();

        // come out from lander TODO: ENCODE ME
        robot.driveMotors(0.4, -0.4, -0.4, 0.4);
        sleep(1200);
        robot.driveMotors(0, 0, 0, 0);

        // turn to realign
        robot.lessBadTurn(0, 0.5);

        // Move left/right depending on mineral position TODO: ENCODE ME
        if (goldMineral == MineralPosition.LEFT) {
            robot.driveMotors(0.5, 0.5, 0.5, 0.5);
            sleep(700);
            robot.driveMotors(0, 0, 0, 0);
        } else if (goldMineral == MineralPosition.RIGHT) {
            robot.driveMotors(-0.5, -0.5, -0.5, -0.5);
            sleep(500);
            robot.driveMotors(0, 0, 0, 0);
        }

        boolean goForCrater = true;
        boolean goForDepot = true;

        if (isSafeAuto()) {
            if (getStartingPosition() == StartingPosition.CRATER) {
                goForDepot = false;
            } else if (getStartingPosition() == StartingPosition.DEPOT) {
                goForCrater = false;
            }
        }

        // if at crater, try to go in
        if (getStartingPosition() == StartingPosition.CRATER) {
            // turn 90
            robot.lessBadTurn(90); //TODO: New plate, don't need turn

            // hit mineral
            robot.driveMotors(-0.8, -0.8, -0.8, -0.8);
            sleep(350);
            // back up a little
            robot.driveMotors(0.8, 0.8, 0.8, 0.8);
            sleep(450);
            robot.driveMotors(0, 0, 0, 0);

            robot.lessBadTurn(0); //TODO: New plate, don't need turn

            // go back to OG position
            if (goldMineral == MineralPosition.LEFT) {
                robot.driveMotors(-0.5, -0.5, -0.5, -0.5);
                sleep(700);
                robot.driveMotors(0, 0, 0, 0);
            } else if (goldMineral == MineralPosition.RIGHT) {
                robot.driveMotors(0.5, 0.5, 0.5, 0.5);
                sleep(500);
                robot.driveMotors(0, 0, 0, 0);
            }

            //Give it a rest boi
            sleep(300);

            if (goForDepot) {
                // Head towards depot diagonally
                robot.driveMotors(1.0, 1.0, 1.0, 1.0);
                sleep(650);
                robot.driveMotors(0, 0, 0, 0);

                // Rotate left by 45 degrees so we're pointed straight
                robot.lessBadTurn(45);

                // Strafe towards wall
                robot.driveMotors(0.8, -0.8, -0.8, 0.8);
                sleep(400);
                robot.driveMotors(0, 0, 0, 0);

<<<<<<< HEAD
            long timeToDepot = 850;
//            // Drive forwards into depot
//            robot.driveMotors(1.0, 1.0, 1.0, 1.0);
//            sleep(timeToDepot);
//            robot.driveMotors(0, 0, 0, 0);

            AutoAligner aligner = new AutoAligner();
            aligner.driveAlignDistanceRobotTime(robot, 1.0, 10, timeToDepot);
=======
                long timeToDepot = 850;
                if (false) {
                    AutoAligner aligner = new AutoAligner();
                    boolean notThereYet = true;
                    ElapsedTime elapsedTime = new ElapsedTime();
                    while (opModeIsActive() && elapsedTime.milliseconds() < timeToDepot) {
                        aligner.driveAlignDistanceRobot(robot, 0.8, 10);
                        idle();
                    }
                } else {
                    // Drive forwards into depot
                    robot.driveMotors(1.0, 1.0, 1.0, 1.0);
                    sleep(timeToDepot);
                    robot.driveMotors(0, 0, 0, 0);
                }
>>>>>>> cf30247b4f04d683af7cb4f0f4bec0db5718477f

                robot.lessBadTurn(135);
            }
        } else if (getStartingPosition() == StartingPosition.DEPOT) {
            // turn 90
            robot.lessBadTurn(90); //TODO: New plate, don't need turn

            // Push mineral
            robot.driveMotors(-0.5, -0.5, -0.5, -0.5);
            sleep(1500);
            robot.driveMotors(0, 0, 0, 0);

            robot.lessBadTurn(0);

            if (goldMineral == MineralPosition.LEFT) {
                robot.driveMotors(-0.5, -0.5, -0.5, -0.5);
                sleep(700);
                robot.driveMotors(0, 0, 0, 0);
            } else if (goldMineral == MineralPosition.RIGHT) {
                robot.driveMotors(0.5, 0.5, 0.5, 0.5);
                sleep(500);
                robot.driveMotors(0, 0, 0, 0);
            }
        }

        ElapsedTime timer = new ElapsedTime();
        robot.latchLeft.setPower(1);
        robot.latchRight.setPower(1);

        if (goForDepot) {
            // Drop team marker
            robot.teamMarker.setPosition(Robot.SERVO_TEAM_MARKER_DEPOSIT);
            sleep(500);

            if (goForCrater) {
                robot.lessBadTurn(getStartingPosition() == StartingPosition.CRATER ? 45 : -45);

<<<<<<< HEAD
        AutoAligner aligner = new AutoAligner();
        // Drive hard towards crater

        aligner.driveAlignDistanceRobotTime(robot, -1.0, 5, 1000);

//        robot.driveMotors(-1, -1, -1, -1);
//        sleep(1000);
//        robot.driveMotors(0, 0, 0, 0);

        // Glide into crater
        aligner.driveAlignDistanceRobotTime(robot, -0.6, 5, 500);

//        robot.driveMotors(-0.6, -0.6, -0.6, -0.6);
//        sleep(500);
//        robot.driveMotors(0, 0, 0, 0);

//        robot.deactivateTfod();
=======
                // Strafe towards wall if at depot to avoid mineral
                robot.driveMotors(0.8, -0.8, -0.8, 0.8);
                sleep(300);
                robot.driveMotors(0, 0, 0, 0);

                // Drive hard towards crater
                robot.driveMotors(-1, -1, -1, -1);
                sleep(1000);
                robot.driveMotors(0, 0, 0, 0);
            }
        }

        if (goForCrater) {
            // Glide into crater
            robot.driveMotors(-0.6, -0.6, -0.6, -0.6);
            sleep(500);
            robot.driveMotors(0, 0, 0, 0);
        }

        robot.deactivateTfod();

>>>>>>> cf30247b4f04d683af7cb4f0f4bec0db5718477f
        robot.teamMarker.setPosition(Robot.SERVO_TEAM_MARKER_HELD);

        // lower latch
        while (opModeIsActive() && timer.seconds() < 7) {
            telemetry.addData("Time", timer.seconds());
            telemetry.update();
            idle();
        }

        robot.latchLeft.setPower(0);
        robot.latchRight.setPower(0);

        // save heading
        PersistentHeading.saveHeading(robot.getHeading());
    }
}
