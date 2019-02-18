package org.firstinspires.ftc.teamcode.opmodes.auto;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.robot.Robot;
import org.firstinspires.ftc.teamcode.utils.MineralPosition;
import org.firstinspires.ftc.teamcode.utils.PersistentHeading;
import org.firstinspires.ftc.teamcode.utils.StartingPosition;

import java.util.Collections;
import java.util.Comparator;
import java.util.Dictionary;
import java.util.HashMap;
import java.util.Map;

import static org.firstinspires.ftc.teamcode.utils.MineralPosition.CENTER;
import static org.firstinspires.ftc.teamcode.utils.MineralPosition.LEFT;
import static org.firstinspires.ftc.teamcode.utils.MineralPosition.RIGHT;

public abstract class AutoMain extends LinearOpMode {

    public abstract StartingPosition getStartingPosition();
    public abstract boolean shouldEndInOtherCrater();

    public int MINERAL_TICKS = 700;
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

        ElapsedTime superTimer = new ElapsedTime();
        superTimer.reset();

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

        int leftCount = 0;
        int centerCount = 0;
        int rightCount = 0;
        int whiteLeftCount = 0;
        int whiteRightCount = 0;
        int whiteCenterCount = 0;

        double goldRightVote = 0;
        double goldLeftVote = 0;
        double goldCenterVote = 0;

        int totalReads = 0;

        while ((Math.abs(robot.latchLeft.getCurrentPosition() - (latchLeftStart-Robot.LATCH_DISTANCE)) > 30 || Math.abs(robot.latchRight.getCurrentPosition() - (latchRightStart-Robot.LATCH_DISTANCE)) > 30) && opModeIsActive()){
            HashMap<MineralPosition, Robot.MineralType> reading = robot.findGoldMineralDifferent();

            if (reading.get(MineralPosition.CENTER) == Robot.MineralType.GOLD) {
//                centerCount++;
                goldCenterVote++;
            } else if (reading.get(LEFT) == Robot.MineralType.GOLD) {
//                leftCount++;
                goldLeftVote++;
            } else if (reading.get(RIGHT) == Robot.MineralType.GOLD) {
//                rightCount++;
                goldRightVote++;
            } else if (reading.get(MineralPosition.CENTER) == Robot.MineralType.SILVER) {
//                whiteCenterCount++;
                goldCenterVote--;
            } else if (reading.get(LEFT) == Robot.MineralType.SILVER) {
//                whiteLeftCount++;
                goldLeftVote--;
            } else if (reading.get(RIGHT) == Robot.MineralType.SILVER) {
//                whiteRightCount++;
                goldRightVote--;
            }
            totalReads++;

//            if (reading == MineralPosition.LEFT) {
//                leftCount++;
//            } else if (reading == MineralPosition.CENTER) {
//                centerCount++;
//            } else if (reading == MineralPosition.RIGHT) {
//                rightCount++;
//            }

            sleep(10);
//            telemetry.addData("raw reading", reading);
            telemetry.addData("gold left vote", goldLeftVote);
            telemetry.addData("gold center vote", goldCenterVote);
            telemetry.addData("gold right vote", goldRightVote);
            telemetry.addData("total reads", totalReads);
            telemetry.update();
            idle();
        }

        sleep(200);

        robot.latchLeft.setPower(0);
        robot.latchRight.setPower(0);

        telemetry.addData("Heading - unlatched", robot.getHeading());
        telemetry.update();

        MineralPosition goldMineral = MineralPosition.RIGHT;

//        if (leftCount > rightCount && leftCount > centerCount) {
//            goldMineral = LEFT;
//        } else if (rightCount > leftCount && rightCount > centerCount) {
//            goldMineral = RIGHT;
//        } else if (centerCount > rightCount && centerCount > leftCount) {
//            goldMineral = CENTER;
//        } else if (whiteLeftCount < whiteCenterCount && whiteLeftCount < whiteRightCount) {
//            goldMineral = LEFT;
//        } else if (whiteRightCount < whiteCenterCount && whiteRightCount < whiteLeftCount) {
//            goldMineral = RIGHT;
//        } else if (whiteCenterCount < whiteLeftCount && whiteCenterCount < whiteRightCount) {
//            goldMineral = CENTER;
//        } else {
//            goldMineral = RIGHT;
//        }

        if (goldLeftVote >= goldRightVote && goldLeftVote >= goldCenterVote) {
            goldMineral = LEFT;
        } else if (goldCenterVote >= goldRightVote && goldCenterVote >= goldLeftVote) {
            if (goldCenterVote == goldRightVote) {
                goldMineral = LEFT;
            } else {
                goldMineral = CENTER;
            }
        } else if (goldRightVote >= goldCenterVote && goldRightVote >= goldLeftVote) {
            goldMineral = RIGHT;
        }
//        if (goldMineral == MineralPosition.UNKNOWN) {
//            if (leftCount > centerCount && leftCount > rightCount) {
//                goldMineral = LEFT;
//            } else if (centerCount > leftCount && centerCount > rightCount) {
//                goldMineral = MineralPosition.CENTER;
//            } else if (rightCount > leftCount && rightCount > centerCount) {
//                goldMineral = RIGHT;
//            } else if (rightCount == 0) {
//                goldMineral = LEFT;
//            } else if (leftCount == 0) {
//                goldMineral = RIGHT;
//            } else if (centerCount == 0) {
//                goldMineral = RIGHT;
//            }
//        }


        telemetry.addData("interpolated gold mineral", goldMineral.name());
        telemetry.update();

//        if (goldMineral == MineralPosition.UNKNOWN) goldMineral = RIGHT;

        int initialTicks = robot.frontLeft.getCurrentPosition();

        // strafe away from lander //TODO: These two shouldn't be different times + ENCODE
        //robot.driveMotors(-1, -1, -1, -1);
        robot.driveTicks(-100, -0.9, -0.9, -0.9, -0.9);
        //sleep(70);
        //robot.driveMotors(0, 0, 0, 0);

//        robot.driveMotors(1, -1, -1, 1);
//        sleep(200);
//        robot.driveMotors(0, 0, 0, 0);
//        robot.driveTicks();

        robot.latchLeft.setPower(0.8);
        robot.latchRight.setPower(1);
        //-Robot.LATCH_DISTANCE + 3000

        robot.latchLeft.setTargetPosition(latchLeftStart - Robot.LATCH_DISTANCE + 6000);
        robot.latchRight.setTargetPosition(latchRightStart - Robot.LATCH_DISTANCE + 6000);

        // turn to realign
        robot.lessBadTurn(0, 0.5);

        if (true) {
            ElapsedTime timer = new ElapsedTime();

            robot.backLeftServo.setPosition(Robot.SENSOR_REV_SERVO_ZERO);

            // Get centered on minerals/lander
            //robot.aligner.centerInCorner(2.0, true);

            // Get close to minerals (away from lander)
            //robot.aligner.driveToDistance(Robot.Direction.RIGHT, Robot.Direction.RIGHT, true, 850, 2.5, true, false);

            robot.driveTicks(650, 0.9, -0.9, -0.9, 0.9);

//            robot.lessBadTurn(0, 0.5);

            // Get in front of cube
            if (goldMineral == LEFT) {
                robot.driveTicks(100 + MINERAL_TICKS + 550, 0.9, 0.9, 0.9, 0.9);
//                robot.driveTicks(200 + MINERAL_TICKS, 0.9, 0.9, 0.9, 0.9);
                //robot.aligner.driveToDistance(Robot.Direction.FORWARD, Robot.Direction.RIGHT, true, 560, 2.0, true);
            } else if (goldMineral == RIGHT) {
                robot.driveTicks(100 - MINERAL_TICKS + 100, -0.9, -0.9, -0.9, -0.9);
//                robot.driveTicks(200 - MINERAL_TICKS, -0.9, -0.9, -0.9, -0.9);
                //robot.aligner.driveToDistance(Robot.Direction.BACKWARD, Robot.Direction.LEFT, true, 560, 2.0, true);
            } else {
                robot.driveTicks(200, 0.9, 0.9, 0.9, 0.9);
            }

            robot.lessBadTurn(0, 0.5);
//            robot.lessBadTurn(-90, 1.5);

            // Hit cube
            if (getStartingPosition() == StartingPosition.DEPOT) {
                if (goldMineral == LEFT) {
                    robot.driveTicks(1650, 0.9, -0.9, -0.9, 0.9);
//                    robot.driveTicks(1650, 1, 1, 1, 1);
                    //robot.aligner.driveToDistance(Robot.Direction.RIGHT, Robot.Direction.LEFT, true, 100, 2.0, true);
                } else if (goldMineral == RIGHT) {
                    robot.driveTicks(1650, 0.9, -0.9, -0.9, 0.9);
//                    robot.driveTicks(1650, 1, 1, 1, 1);
                    //robot.aligner.driveToDistance(Robot.Direction.RIGHT, Robot.Direction.RIGHT, true, 100, 2.0, true);
                } else {
                    robot.lessBadTurn(-90);
                    robot.driveTicks(1650, 1, 1, 1, 1);
                    robot.lessBadTurn(0, 1.5);
//                    robot.aligner.driveToDistance(Robot.Direction.RIGHT, Robot.Direction.LEFT, true, 200, 1.5, true);
                }
                robot.lessBadTurn(0, 0.5);
//                robot.lessBadTurn(0, 1.5);

                if (goldMineral == LEFT) {
//                    robot.driveTicks(-MINERAL_TICKS, -0.9, -0.9, -0.9, -0.9);
                    robot.driveTicks(100 - MINERAL_TICKS - 100, -0.9, -0.9, -0.9, -0.9);
//                    robot.driveTicks(100 - MINERAL_TICKS, -0.9, -0.9, -0.9, -0.9);
                } else if (goldMineral == RIGHT) {
//                    robot.driveTicks(MINERAL_TICKS, 0.9, 0.9, 0.9, 0.9);
                    robot.driveTicks(100 + MINERAL_TICKS, 0.9, 0.9, 0.9, 0.9);
//                    robot.driveTicks(100 + MINERAL_TICKS, 0.9, 0.9, 0.9, 0.9);
                }

                //if (goldMineral != MineralPosition.CENTER) {
//                    robot.aligner.centerInCorner(2, true);
//                    if (goldMineral == MineralPosition.LEFT) {
//                        robot.driveTicks(-MINERAL_TICKS, -1.0);
//                    } else {
//                        robot.driveTicks(MINERAL_TICKS, 1.0);
//                    }
//                }

                robot.teamMarker.setPosition(Robot.SERVO_TEAM_MARKER_DEPOSIT);
                sleep(500);

                if (!shouldEndInOtherCrater()) {
                    robot.lessBadTurn(-45);
                } else {
                    robot.lessBadTurn(45);
                }
                robot.setupSimpleServos(Robot.Direction.RIGHT);

//                robot.aligner.driveToDistance(Robot.Direction.RIGHT, Robot.Direction.RIGHT, false, 100, 1.5, true);
                if (!shouldEndInOtherCrater()) {
                    robot.driveTicks(600,0.9, -0.9, -0.9, 0.9);
                } else {
                    robot.driveTicks(800,0.9, -0.9, -0.9, 0.9);
                }

            }
            if (getStartingPosition() == StartingPosition.CRATER) {
                if (goldMineral == LEFT) {
                    robot.driveTicks(800, 0.9, -0.9, -0.9, 0.9);
                    robot.driveTicks(-650, -0.9, 0.9, 0.9, -0.9);
                    //robot.aligner.driveToDistance(Robot.Direction.RIGHT, Robot.Direction.LEFT, true, 320, 1.0, true);
                    //robot.aligner.driveToDistance(Robot.Direction.RIGHT, Robot.Direction.LEFT, true, 580, 1.0, true);
                } else if (goldMineral == RIGHT) {
                    robot.driveTicks(800, 0.9, -0.9, -0.9, 0.9);
                    robot.driveTicks(-650, -0.9, 0.9, 0.9, -0.9);
                    //robot.aligner.driveToDistance(Robot.Direction.RIGHT, Robot.Direction.RIGHT, true, 320, 1.0, true);
                    //robot.aligner.driveToDistance(Robot.Direction.RIGHT, Robot.Direction.RIGHT, true, 580, 1.0, true);
                } else {
                    robot.lessBadTurn(-90);
                    robot.driveTicks(800,0.9, 0.9, 0.9, 0.9);
                    robot.driveTicks(-250,-0.9, -0.9, -0.9, -0.9);
                    robot.lessBadTurn(0, 1.5);
                }

                robot.lessBadTurn(0, 1.0);

                if (goldMineral == LEFT) {
                    robot.driveTicks(2000 - MINERAL_TICKS - 100, 0.9, 0.9, 0.9, 0.9);
                } else if (goldMineral == RIGHT) {
                    robot.driveTicks(2000 + MINERAL_TICKS, 0.9, 0.9, 0.9, 0.9);
                } else {
                    robot.driveTicks(2000, 0.9, 0.9, 0.9, 0.9);
                }
//                robot.aligner.driveToDistance(Robot.Direction.FORWARD, Robot.Direction.RIGHT, true, 300, 2.0, true);

//                double angle = robot.getHeading();
//                double relativeError = angle / 45;
//                robot.frontRightServo.setPosition(Robot.SENSOR_SERVO_FULL + (relativeError - 1) * (Robot.SENSOR_SERVO_FULL-Robot.SENSOR_SERVO_HALF));
//                robot.backRightServo.setPosition(Robot.SENSOR_REV_SERVO_FULL + (1 - relativeError) * (Robot.SENSOR_REV_SERVO_FULL-Robot.SENSOR_REV_SERVO_HALF));
//                sleep(200);
                robot.setupSimpleServos(Robot.Direction.RIGHT);

                robot.lessBadTurn(45);

                robot.driveTicks(250, 0.9, -0.9, -0.9, 0.9);

                sleep(4000);

                timer.reset();
//                robot.driveTicks(3000, 0.9, 0.9, 0.9, 0.9);
                while (opModeIsActive()) {
//                    robot.driveMotors(0.9, 0.9, 0.9, 0.9);
                    robot.aligner.driveAlignDistance(0.9, 100, false);
                    if (timer.seconds() > 1.05) break;
                    idle();
                }
//                while (opModeIsActive() && robot.rangeFrontLeft.cmUltrasonic() > 10) {
//                    telemetry.addData("front left", robot.frontLeft.getCurrentPosition());
//                    telemetry.update();
//                    robot.aligner.driveAlignDistance(0.9, 100, false);
//                    if (timer.seconds() > 1.1) break;
//                    idle();
//                }
                robot.driveMotors(0, 0, 0, 0);

                // CHANGE?
//                robot.aligner.driveToDistance(Robot.Direction.RIGHT, Robot.Direction.RIGHT, false, 100, 0.5, true);


                robot.teamMarker.setPosition(Robot.SERVO_TEAM_MARKER_DEPOSIT);
                sleep(500);
            }

            robot.latchLeft.setPower(0.0);
            robot.latchRight.setPower(0.0);

            timer.reset();
//            if (isSafeAuto()) {
//                robot.driveTicks(3000, 0.9, 0.9, 0.9, 0.9);
//            } else {
//                robot.driveTicks(3000, -0.9, -0.9, -0.9, -0.9);
//            }
            while (opModeIsActive()) {
                if (!shouldEndInOtherCrater()) {
                    robot.aligner.driveAlignDistance(-0.9, 100, false);
//                    robot.driveMotors(-0.9, -0.9, -0.9, -0.9);
                } else {
                    robot.aligner.driveAlignDistance(0.9, 100, false);
//                    robot.driveMotors(0.9, 0.9, 0.9, 0.9);
                }
                if (getStartingPosition() == StartingPosition.CRATER) {
                    if (timer.seconds() > 0.95) break;
                } else {
                    if (timer.seconds() > 0.7) break;
                }

                idle();
            }

            if (getStartingPosition() == StartingPosition.DEPOT) {
                if (!shouldEndInOtherCrater()) {
                    robot.lessBadTurn(135, 2.5);
                }
            } else {
                robot.lessBadTurn(-135, 2.5);
            }

            if (superTimer.seconds() <= 25) {
                robot.lenny.setPower(1.0);
                sleep(3000);
                robot.lenny.setPower(0.0);
            }

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
        if (goldMineral == LEFT) {
            robot.driveMotors(0.5, 0.5, 0.5, 0.5);
            sleep(700);
            robot.driveMotors(0, 0, 0, 0);
        } else if (goldMineral == RIGHT) {
            robot.driveMotors(-0.5, -0.5, -0.5, -0.5);
            sleep(500);
            robot.driveMotors(0, 0, 0, 0);
        }

        boolean goForCrater = true;
        boolean goForDepot = true;

        if (shouldEndInOtherCrater()) {
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
            if (goldMineral == LEFT) {
                robot.driveMotors(-0.5, -0.5, -0.5, -0.5);
                sleep(700);
                robot.driveMotors(0, 0, 0, 0);
            } else if (goldMineral == RIGHT) {
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

            if (goldMineral == LEFT) {
                robot.driveMotors(-0.5, -0.5, -0.5, -0.5);
                sleep(700);
                robot.driveMotors(0, 0, 0, 0);
            } else if (goldMineral == RIGHT) {
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
