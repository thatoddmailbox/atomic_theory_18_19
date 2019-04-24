package org.firstinspires.ftc.teamcode.opmodes.auto;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.blackbox.LogContext;
import org.firstinspires.ftc.teamcode.blackbox.MatchPhase;
import org.firstinspires.ftc.teamcode.blackbox.sensors.WrappedLynxModule;
import org.firstinspires.ftc.teamcode.robot.Robot;
import org.firstinspires.ftc.teamcode.utils.LynxPumperRunnable;
import org.firstinspires.ftc.teamcode.utils.Direction;
import org.firstinspires.ftc.teamcode.utils.MineralPosition;
import org.firstinspires.ftc.teamcode.utils.OptionsManager;
import org.firstinspires.ftc.teamcode.utils.PersistentHeading;
import org.firstinspires.ftc.teamcode.utils.RobotFeature;
import org.firstinspires.ftc.teamcode.utils.StartingPosition;
import org.xmlpull.v1.XmlPullParserException;

import java.io.IOException;
import java.util.HashMap;

public abstract class AutoMain extends LinearOpMode {

    public abstract StartingPosition getStartingPosition();

    public int MINERAL_TICKS = 600;
    @Override
    public void runOpMode() throws InterruptedException {
        // When init is pressed make an instance of Robot
        telemetry.addData("Status", "Starting...");
        telemetry.update();

        PersistentHeading.clearSavedHeading();
        try (Robot robot = new Robot(MatchPhase.AUTONOMOUS, this, new RobotFeature[]{
                RobotFeature.CAMERA
        })) {
            boolean useUltrasonic = OptionsManager.getBooleanSetting("useUltrasonic");
            boolean endInOtherCrater = OptionsManager.getBooleanSetting("alternateCrater");
            double timeDelay = OptionsManager.getDoubleSetting("timeDelay");
            boolean endNom = OptionsManager.getBooleanSetting("endNom");
            boolean simpleAuto = OptionsManager.getBooleanSetting("simpleAuto");

            // print telemetry
            HashMap<String, String> displayList = OptionsManager.getDisplayList();
            telemetry.addData("Status", "Ready to go");
            telemetry.addData("Starting position", getStartingPosition());
            for (HashMap.Entry<String, String> entry : displayList.entrySet()) {
                telemetry.addData(entry.getKey(), entry.getValue());
            }
            telemetry.update();

            robot.teamMarker.setPosition(Robot.SERVO_TEAM_MARKER_HELD);
            robot.resetHeading();

            // Wait for play button, then begin timer
            waitForStart();
            robot.handleMatchStart();

            ElapsedTime superTimer = new ElapsedTime();
            superTimer.reset();

            telemetry.addData("Status", "Running");
            telemetry.update();

            robot.activateTfod();

            /*

            UNLATCH
             - Different set powers account for weird motor discrepancy
             - latchLeft and latchRight start reset encoder values

             */
            int latchLeftStart = robot.latchLeft.getCurrentPosition();
            int latchRightStart = robot.latchRight.getCurrentPosition();

            robot.latchLeft.setPower(-0.8);
            robot.latchRight.setPower(-1);

            robot.latchLeft.setTargetPosition(latchLeftStart - Robot.LATCH_DISTANCE);
            robot.latchRight.setTargetPosition(latchRightStart - Robot.LATCH_DISTANCE);


            // Set camera votes and reads to 0
            double goldCenterVote = 0;
            int totalCenterVotes = 0;
            double goldLeftVote = 0;
            int totalLeftVotes = 0;
            double goldRightVote = 0;
            int totalRightVotes = 0;

            int totalReads = 0;

            /*

            READ SAMPLE
            - Record votes for mineral position
            - Begins taking readings until latch approaches bottom (30 ticks away)

             */
            try (LogContext context = robot.logSession.createContext("unlatch and sample")) {
                while ((Math.abs(robot.latchLeft.getCurrentPosition() - (latchLeftStart - Robot.LATCH_DISTANCE)) > 30 || Math.abs(robot.latchRight.getCurrentPosition() - (latchRightStart - Robot.LATCH_DISTANCE)) > 30) && opModeIsActive()) {
                    HashMap<MineralPosition, Float> reading = robot.findGoldMineralDifferent();

                    if (reading.containsKey(MineralPosition.CENTER)) {
                        goldCenterVote += reading.get(MineralPosition.CENTER);
                        totalCenterVotes += 1;
                    }
                    if (reading.containsKey(MineralPosition.LEFT)) {
                        goldLeftVote += reading.get(MineralPosition.LEFT);
                        totalLeftVotes += 1;
                    }
                    if (reading.containsKey(MineralPosition.RIGHT)) {
                        goldRightVote += reading.get(MineralPosition.RIGHT);
                        totalRightVotes += 1;
                    }

                    totalReads++;

                    // Log votes to telemetry
                    sleep(10);
                    telemetry.addData("gold left vote", goldLeftVote / totalLeftVotes);
                    telemetry.addData("gold center vote", goldCenterVote / totalCenterVotes);
                    telemetry.addData("gold right vote", goldRightVote / totalRightVotes);
                    telemetry.addData("total reads", totalReads);
                    telemetry.update();
                    idle();
                }

                context.setFact("avg gold left vote", goldLeftVote / totalLeftVotes);
                context.setFact("avg gold center vote", goldCenterVote / totalCenterVotes);
                context.setFact("avg gold right vote", goldRightVote / totalRightVotes);
                context.setFact("total reads", totalReads);
            }

            sleep(100);

            robot.latchLeft.setPower(0);
            robot.latchRight.setPower(0);

            telemetry.addData("Heading - unlatched", robot.getHeading());
            telemetry.update();

            /*

            SAMPLE DECISION LOGIC
            - Side with the highest gold vote average is gold

             */
            goldCenterVote /= totalCenterVotes;
            goldLeftVote /= totalLeftVotes;
            goldRightVote /= totalRightVotes;

            MineralPosition goldMineral = MineralPosition.RIGHT;

            if (goldLeftVote >= goldRightVote && goldLeftVote >= goldCenterVote) {
                goldMineral = MineralPosition.LEFT;
            } else if (goldCenterVote >= goldRightVote && goldCenterVote >= goldLeftVote) {
                if (goldCenterVote != goldRightVote) {
                    goldMineral = MineralPosition.CENTER;
                }
            }

            telemetry.addData("final gold mineral prediction:", goldMineral.name());
            telemetry.update();


            // Drive backwards to unlatch
            robot.driveTicks(-100, -0.9, -0.9, -0.9, -0.9);

            // Drive latch downwards once unlatched (all the way to bottom)
//            robot.latchLeft.setPower(0.8);
//            robot.latchRight.setPower(1);
//
//            robot.latchLeft.setTargetPosition(latchLeftStart);
//            robot.latchRight.setTargetPosition(latchRightStart);

            // Take half a second to face the minerals
            robot.turn(0, 0.5);

            ElapsedTime timer = new ElapsedTime();

            robot.frontRightServo.setPosition(Robot.SENSOR_SERVO_HALF);
            robot.backRightServo.setPosition(Robot.SENSOR_REV_SERVO_HALF);
            robot.backLeftServo.setPosition(Robot.SENSOR_REV_SERVO_ZERO);

            // Get away from lander
            robot.driveTicks(1500, 1, -0.6, -1, 0.6);

            robot.turn(0, 0.5);

            // Get in front of cube
            if (useUltrasonic) {
                MINERAL_TICKS += 50;
                robot.rangeFrontRight.cmUltrasonic();
                robot.rangeBackRight.cmUltrasonic();

                robot.aligner.centerInCorner(2.0, true);

                if (goldMineral == MineralPosition.LEFT) {
                    robot.driveTicks(MINERAL_TICKS, 0.9, 0.9, 0.9, 0.9);
                } else if (goldMineral == MineralPosition.RIGHT) {
                    robot.driveTicks(-MINERAL_TICKS, -0.9, -0.9, -0.9, -0.9);
                }
            } else {
                double p = 0.9;
                if (simpleAuto) p = 0.5;
                // TODO: change me at competition - this is "very sketch" and probably relies on our lander being dumb
                if (goldMineral == MineralPosition.LEFT) {
                    if (getStartingPosition() == StartingPosition.CRATER) {
                        //250 +
                        robot.driveTicks(750 + MINERAL_TICKS, p, p, p, p);
                    } else {
                        //robot.driveTicks(MINERAL_TICKS, 0.9, 0.9, 0.9, 0.9);
                        robot.driveTicks(750 + MINERAL_TICKS, p, p, p, p);
                    }
                } else if (goldMineral == MineralPosition.RIGHT) {
                    if (getStartingPosition() == StartingPosition.CRATER) {
//                        200 -
//                        robot.driveTicks(750 - MINERAL_TICKS, -p, -p, -p, -p);
                    } else {
                        //robot.driveTicks(300 - MINERAL_TICKS, -0.9, -0.9, -0.9, -0.9);
//                        robot.driveTicks(750 - MINERAL_TICKS, -p, -p, -p, -p);
                    }
                } else {
                    //95
                    if (getStartingPosition() == StartingPosition.CRATER) {
                        robot.driveTicks(750, p, p, p, p);
                    } else {
                        //robot.driveTicks(0, 0.9, 0.9, 0.9, 0.9);
                        robot.driveTicks(650, p, p, p, p);
                    }
                }
            }

            // Turn to drive head on into cube
            robot.turn(-90, 2.0);

            if (getStartingPosition() == StartingPosition.DEPOT) {
                sleep(Math.round(timeDelay * 1000));

                // Hit cube
                if (goldMineral == MineralPosition.CENTER) {
                    robot.driveTicks(1900, 0.8, 0.8, 0.8, 0.8);
                } else {
                    robot.driveTicks(1900, 0.8, 0.8, 0.8, 0.8);
                }
                robot.turn(0, 1.5);

                // Recenter
                if (goldMineral == MineralPosition.LEFT) {
//                    robot.driveTicks(-MINERAL_TICKS, -1, -1, -1, -1);
                    robot.driveTicks(-MINERAL_TICKS + 100, -1, -1, -1, -1);
                } else if (goldMineral == MineralPosition.RIGHT) {
//                    robot.driveTicks(MINERAL_TICKS, 1, 1, 1, 1);
                    robot.driveTicks(MINERAL_TICKS - 200, 1, 1, 1, 1);
                }

                // Drop team marker
                robot.teamMarker.setPosition(Robot.SERVO_TEAM_MARKER_DEPOSIT);
                robot.setupSimpleServos(Direction.RIGHT);

                sleep(400);

                // Turn and strafe into wall
                if (!endInOtherCrater) {
                    robot.turn(-45, 1.5);
                    robot.driveTicks(1000, 0.9, -0.9, -0.9, 0.9);
                } else {
                    robot.turn(45, 1.5);
                    robot.driveTicks(1600, 0.9, -0.9, -0.9, 0.9);
                }
            } else if (getStartingPosition() == StartingPosition.CRATER) {
                // Hit cube and back up again
                if (simpleAuto) {
                    robot.driveTicks(900, 0.5, 0.5, 0.5, 0.5);
//                    robot.driveTicks(-500, -0.5, -0.5, -0.5, -0.5);
//                    robot.lenny.setPower(1.0);
//               `     sleep(2000);
//                    robot.lenny.setPower(0.0);
                    return;
                } else {
                    if (goldMineral == MineralPosition.LEFT) {
                        robot.driveTicks(1100, 1, 1, 1, 1);
                    } else if (goldMineral == MineralPosition.RIGHT) {
                        robot.driveTicks(1000, 1, 1, 1, 1);
                    } else {
                        robot.driveTicks(900, 1, 1, 1, 1);
                    }
                }

                robot.driveTicks(-350, -1, -1, -1, -1);

                robot.turn(0, 1.5);

                // Drive to wall
                if (goldMineral == MineralPosition.LEFT) {
                    robot.driveTicks(1800 - MINERAL_TICKS, 1.0, 1.0, 1.0, 1.0);
                } else if (goldMineral == MineralPosition.RIGHT) {
                    robot.driveTicks(1800 + MINERAL_TICKS, 1.0, 1.0, 1.0, 1.0);
                } else {
                    robot.driveTicks(1800, 1.0, 1.0, 1.0, 1.0);
                }

                robot.setupSimpleServos(Direction.RIGHT);

                // Turn and strafe to wall
                robot.turn(45, 1.5);

                robot.driveTicks(1300, 1, -0.7, -1, 0.7);

                // Possible wait for alliance to do their thing
                sleep(Math.round(timeDelay * 1000));

                timer.reset();

//                if (useUltrasonic) {
                robot.aligner.driveAlignDistanceTicks(0.9, 90, 2300, false);
//                } else {
//                    robot.driveTicks(2300, 0.9, 0.9, 0.9, 0.9);
//                }

                // turn to place team marker
                robot.turn(90, 1.0);

                robot.teamMarker.setPosition(Robot.SERVO_TEAM_MARKER_DEPOSIT);
                sleep(400);

                robot.turn(45, 1.0);
            }

            robot.latchLeft.setPower(0.0);
            robot.latchRight.setPower(0.0);

            timer.reset();

            robot.driveTicks(300, 1, -1, -1, 1);

            // Head back to crater
            if (getStartingPosition() == StartingPosition.CRATER) {
                if (useUltrasonic) {
                    robot.aligner.driveAlignDistanceTicks(-0.9, 100, -2200, false);
                } else {
                    robot.driveTicks(-2200, -0.9, -0.9, -0.9, -0.9);
                }
            } else {
                if (!endInOtherCrater) {
                    if (useUltrasonic) {
                        robot.aligner.driveAlignDistanceTicks(-0.9, 100, -2400, false);
                    } else {
                        robot.driveTicks(-2400, -0.9, -0.9, -0.9, -0.9);
                    }
                } else {
                    if (useUltrasonic) {
                        robot.aligner.driveAlignDistanceTicks(0.9, 100, 2400, false);
                    } else {
                        robot.driveTicks(2400, 0.9, 0.9, 0.9, 0.9);
                    }
                }
            }

            // Turn and unfurl arm
            double targetHeading = 0;
            if (getStartingPosition() == StartingPosition.DEPOT) {
                if (!endInOtherCrater) {
                    robot.turn(45, 1.5);
                    robot.turn(135, 2);
                    targetHeading = 135;
                }
            } else {
                robot.turn(-45, 1.5);
                robot.turn(-135, 2);
                targetHeading = -135;
            }

            if (!useUltrasonic) {
                if (getStartingPosition() == StartingPosition.DEPOT) {
                    if (endInOtherCrater) {
                        robot.driveTicks(300, 1, -1, -1, 1);
                    } else {
                        robot.driveTicks(-300, -1, 1, 1, -1);
                    }
                } else {
                    robot.driveTicks(-300, -1, 1, 1, -1);
                }
            }

            double currentHeading = robot.imu.getHeading();

            // Guarantee accurate heading to prevent major penalty/desampling in arm deployment
            if (targetHeading != 0 && getStartingPosition() == StartingPosition.DEPOT) {
                if (Math.abs(currentHeading - targetHeading) > 8) {
                    return;
                }
            }

//            if (getStartingPosition() == StartingPosition.DEPOT) {
                if (superTimer.seconds() <= 26.5) {
                    robot.lenny.setPower(1.0);
                    sleep(2000);
                    robot.lenny.setPower(0.0);

                    long moveTime = Math.max((long) (30 - superTimer.seconds() - 1) * 1000, 3000);

                    telemetry.addData("moveTime", moveTime);
                    telemetry.update();

                    robot.george.setPower(1);
                    sleep(moveTime);
                    robot.george.setPower(0.0);
                } else if (superTimer.seconds() <= 29.15 && endNom) {
                    robot.driveMotors(1, -1, -1, 1);
                    sleep(250);
                    robot.driveMotors(0, 0, 0, 0);
                    robot.driveMotors(0.4, 0.4, 0.4, 0.4);
                    sleep(600);
                    robot.driveMotors(0, 0, 0, 0);
                }
//            }
//            else {
//                robot.driveMotors(1, -1, -1, 1);
//                sleep(250);
//                robot.driveMotors(0, 0, 0, 0);
//                robot.driveMotors(-0.4, -0.4, -0.4, -0.4);
//                sleep(700);
//                robot.driveMotors(0, 0, 0, 0);
//            }

            // Save heading
            PersistentHeading.saveHeading(robot.getHeading());
        }
    }
}
