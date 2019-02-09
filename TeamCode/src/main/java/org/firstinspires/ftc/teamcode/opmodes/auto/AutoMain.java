package org.firstinspires.ftc.teamcode.opmodes.auto;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.robot.Robot;
import org.firstinspires.ftc.teamcode.utils.MineralPosition;
import org.firstinspires.ftc.teamcode.utils.PersistentHeading;
import org.firstinspires.ftc.teamcode.utils.StartingPosition;

public abstract class AutoMain extends LinearOpMode {

    public abstract StartingPosition getStartingPosition();
    public abstract boolean isSafeAuto();

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

        robot.activateTfod();

        // unlatch
        int latchLeftStart = robot.latchLeft.getCurrentPosition();
        int latchRightStart = robot.latchRight.getCurrentPosition();

        robot.latchLeft.setPower(-0.8);
        robot.latchRight.setPower(-1);

        robot.latchLeft.setTargetPosition(latchLeftStart-Robot.LATCH_DISTANCE);
        robot.latchRight.setTargetPosition(latchRightStart-Robot.LATCH_DISTANCE);

        while ((Math.abs(robot.latchLeft.getCurrentPosition() - (latchLeftStart-Robot.LATCH_DISTANCE)) > 30 || Math.abs(robot.latchRight.getCurrentPosition() - (latchRightStart-Robot.LATCH_DISTANCE)) > 30) && opModeIsActive()){
            sleep(10);
            idle();
        }

        sleep(200);

        robot.latchLeft.setPower(0);
        robot.latchRight.setPower(0);

        telemetry.addData("Heading - unlatched", robot.getHeading());
        telemetry.update();

        MineralPosition goldMineral = robot.findGoldMineralDifferent();

        telemetry.addData("gold mineral", goldMineral.name());
        telemetry.update();

        // strafe away from lander //TODO: These two shouldn't be different times + ENCODE
        robot.driveMotors(-1, -1, -1, -1);
        sleep(70);
        robot.driveMotors(0, 0, 0, 0);

        robot.driveMotors(1, -1, -1, 1);
        sleep(200);
        robot.driveMotors(0, 0, 0, 0);

        robot.latchLeft.setPower(0.8);
        robot.latchRight.setPower(1);
        robot.latchLeft.setTargetPosition(latchLeftStart-Robot.LATCH_DISTANCE + 3000);
        robot.latchRight.setTargetPosition(latchRightStart-Robot.LATCH_DISTANCE + 3000);

        // turn to realign
        robot.lessBadTurn(0, 0.5);

        // NEW AUTO (UNTESTED)
        if (true) {
            ElapsedTime timer = new ElapsedTime();

            robot.backLeftServo.setPosition(Robot.SENSOR_REV_SERVO_ZERO);

            // Get centered on minerals/lander
            robot.aligner.centerInCorner(2.0, true);

            // Get close to minerals (away from lander)
            robot.aligner.driveToDistance(Robot.Direction.RIGHT, Robot.Direction.RIGHT, true, 850, 2.5, true, false);

            // Get in front of cube
            if (goldMineral == MineralPosition.LEFT) {
                robot.aligner.driveToDistance(Robot.Direction.FORWARD, Robot.Direction.RIGHT, true, 560, 2.0, true);
            } else if (goldMineral == MineralPosition.RIGHT) {
                robot.aligner.driveToDistance(Robot.Direction.BACKWARD, Robot.Direction.LEFT, true, 560, 2.0, true);
            }

            robot.lessBadTurn(0, 0.5);

            // Hit cube
            if (getStartingPosition() == StartingPosition.DEPOT) {
                if (goldMineral == MineralPosition.LEFT) {
                    robot.aligner.driveToDistance(Robot.Direction.RIGHT, Robot.Direction.LEFT, true, 100, 2.0, true);
                } else if (goldMineral == MineralPosition.RIGHT) {
                    robot.aligner.driveToDistance(Robot.Direction.RIGHT, Robot.Direction.RIGHT, true, 100, 2.0, true);
                } else {
                    robot.lessBadTurn(-90);
                    robot.driveMotors(1.0, 1.0, 1.0, 1.0);
                    sleep(1500);
                    robot.driveMotors(0,0,0 ,0);
                    robot.lessBadTurn(0, 1.5);
//                    robot.aligner.driveToDistance(Robot.Direction.RIGHT, Robot.Direction.LEFT, true, 200, 1.5, true);
                }
                robot.lessBadTurn(0, 0.5);

                if (goldMineral != MineralPosition.CENTER) {
                    robot.aligner.centerInCorner(2, true);
                }
                robot.teamMarker.setPosition(Robot.SERVO_TEAM_MARKER_DEPOSIT);
                sleep(500);

                if (!isSafeAuto()) {
                    robot.lessBadTurn(-45);
                } else {
                    robot.lessBadTurn(45);
                }
                robot.setupSimpleServos(Robot.Direction.RIGHT);

                robot.aligner.driveToDistance(Robot.Direction.RIGHT, Robot.Direction.RIGHT, false, 100, 1.0, true);
            }
            if (getStartingPosition() == StartingPosition.CRATER) {
                if (goldMineral == MineralPosition.LEFT) {
                    robot.aligner.driveToDistance(Robot.Direction.RIGHT, Robot.Direction.LEFT, true, 320, 1.0, true);
                    robot.aligner.driveToDistance(Robot.Direction.RIGHT, Robot.Direction.LEFT, true, 580, 1.0, true);
                } else if (goldMineral == MineralPosition.RIGHT) {
                    robot.aligner.driveToDistance(Robot.Direction.RIGHT, Robot.Direction.RIGHT, true, 320, 1.0, true);
                    robot.aligner.driveToDistance(Robot.Direction.RIGHT, Robot.Direction.RIGHT, true, 580, 1.0, true);
                } else {
                    robot.lessBadTurn(-90);
                    robot.driveMotors(1.0, 1.0, 1.0, 1.0);
                    sleep(300);
                    robot.driveMotors(0,0,0 ,0);
                    robot.driveMotors(-1.0, -1.0, -1.0, -1.0);
                    sleep(400);
                    robot.driveMotors(0,0,0 ,0);
                    robot.lessBadTurn(0, 1.5);
                }

                robot.aligner.driveToDistance(Robot.Direction.FORWARD, Robot.Direction.RIGHT, true, 300, 2.0, true);

//                double angle = robot.getHeading();
//                double relativeError = angle / 45;
//                robot.frontRightServo.setPosition(Robot.SENSOR_SERVO_FULL + (relativeError - 1) * (Robot.SENSOR_SERVO_FULL-Robot.SENSOR_SERVO_HALF));
//                robot.backRightServo.setPosition(Robot.SENSOR_REV_SERVO_FULL + (1 - relativeError) * (Robot.SENSOR_REV_SERVO_FULL-Robot.SENSOR_REV_SERVO_HALF));
//                sleep(200);
                robot.setupSimpleServos(Robot.Direction.RIGHT);

                robot.lessBadTurn(45);

                timer.reset();
                while (opModeIsActive()) {
                    robot.aligner.driveAlignDistance(0.85, 100, false);
                    if (timer.seconds() > 2.1) break;
                    idle();
                }
                robot.driveMotors(0, 0, 0, 0);

                robot.aligner.driveToDistance(Robot.Direction.RIGHT, Robot.Direction.RIGHT, false, 100, 1.0, true);

                robot.teamMarker.setPosition(Robot.SERVO_TEAM_MARKER_DEPOSIT);
                sleep(500);
            }

            robot.latchLeft.setPower(0.0);
            robot.latchRight.setPower(0.0);

            timer.reset();
            while (opModeIsActive()) {
                if (!isSafeAuto()) {
                    robot.aligner.driveAlignDistance(-0.85, 100, false);
                } else {
                    robot.aligner.driveAlignDistance(0.85, 100, false);
                }
                if (timer.seconds() > 1.1) break;
                idle();
            }

            if (getStartingPosition() == StartingPosition.DEPOT) {
                if (!isSafeAuto()) {
                    robot.lessBadTurn(135);
                }
            } else {
                robot.lessBadTurn(-135);
            }
            robot.lenny.setPower(1.0);
            sleep(4000);
            robot.lenny.setPower(0.0);

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

        //MineralPosition goldMineral = robot.findGoldMineralDifferent();
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

                long timeToDepot = 850;
    //            // Drive forwards into depot
                robot.driveMotors(1.0, 1.0, 1.0, 1.0);
                sleep(timeToDepot);
                robot.driveMotors(0, 0, 0, 0);

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
