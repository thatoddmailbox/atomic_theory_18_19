package org.firstinspires.ftc.teamcode.opmodes.auto;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.blackbox.MatchPhase;
import org.firstinspires.ftc.teamcode.robot.Robot;
import org.firstinspires.ftc.teamcode.utils.Direction;
import org.firstinspires.ftc.teamcode.utils.MineralPosition;
import org.firstinspires.ftc.teamcode.utils.PersistentHeading;
import org.firstinspires.ftc.teamcode.utils.StartingPosition;

import java.util.HashMap;

public abstract class AutoMain extends LinearOpMode {

    public abstract StartingPosition getStartingPosition();
    public abstract boolean shouldEndInOtherCrater();

    public int MINERAL_TICKS = 700;
    @Override
    public void runOpMode() throws InterruptedException {

        //When init is pressed  make an instance of Robot
        telemetry.addData("Status", "Starting...");
        telemetry.update();

        PersistentHeading.clearSavedHeading();
        try (Robot robot = new Robot(MatchPhase.AUTONOMOUS, this, true)) {
            //Once robot has inited, telemetrize ready and move team marker
            telemetry.addData("Status", "Ready to go");
            telemetry.addData("Starting position", getStartingPosition());
            telemetry.update();

            robot.teamMarker.setPosition(Robot.SERVO_TEAM_MARKER_HELD);
            robot.resetHeading();

            //Wait for play button, then begin timer
            waitForStart();

            ElapsedTime superTimer = new ElapsedTime();
            superTimer.reset();

            telemetry.addData("Status", "Running");
            telemetry.update();

            robot.activateTfod();

            /*

            UNLATCH
             - Different set powers account for weird motor discrepency
             - latchLeft and latchRight start reset encoder values

             */
            int latchLeftStart = robot.latchLeft.getCurrentPosition();
            int latchRightStart = robot.latchRight.getCurrentPosition();

            robot.latchLeft.setPower(-0.8);
            robot.latchRight.setPower(-1);

            robot.latchLeft.setTargetPosition(latchLeftStart - Robot.LATCH_DISTANCE);
            robot.latchRight.setTargetPosition(latchRightStart - Robot.LATCH_DISTANCE);


            // Reset camera votes and reads to 0
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

                //Log votes to telemetry
                sleep(10);
                telemetry.addData("gold left vote", goldLeftVote / totalLeftVotes);
                telemetry.addData("gold center vote", goldCenterVote / totalCenterVotes);
                telemetry.addData("gold right vote", goldRightVote / totalRightVotes);
                telemetry.addData("total reads", totalReads);
                telemetry.update();
                idle();
            }

            sleep(200);

            robot.latchLeft.setPower(0);
            robot.latchRight.setPower(0);

            telemetry.addData("Heading - unlatched", robot.getHeading());
            telemetry.update();

            /*

            SAMPLE DECISION LOGIC
            - Simple logic, side with the most gold counts is gold

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

            //drive latch downwards once unlatched (all the way to bottom)
            robot.latchLeft.setPower(0.8);
            robot.latchRight.setPower(1);

            robot.latchLeft.setTargetPosition(latchLeftStart);
            robot.latchRight.setTargetPosition(latchRightStart);

            //Take half a second to face the minerals
            robot.turn(0, 0.5);

            ElapsedTime timer = new ElapsedTime();

            robot.backLeftServo.setPosition(Robot.SENSOR_REV_SERVO_ZERO);

            // Get away from lander
            robot.driveTicks(650, 0.9, -0.9, -0.9, 0.9);

            // Get in front of cube
            if (goldMineral == MineralPosition.LEFT) {
//                robot.driveTicks(100 + MINERAL_TICKS + 550, 0.9, 0.9, 0.9, 0.9);
                robot.driveTicks(150 + MINERAL_TICKS, 0.9, 0.9, 0.9, 0.9);
                //robot.aligner.driveToDistance(Direction.FORWARD, Direction.RIGHT, true, 560, 2.0, true);
            } else if (goldMineral == MineralPosition.RIGHT) {
//                robot.driveTicks(100 - MINERAL_TICKS + 100, -0.9, -0.9, -0.9, -0.9);
                robot.driveTicks(150 - MINERAL_TICKS, -0.9, -0.9, -0.9, -0.9);
                //robot.aligner.driveToDistance(Direction.BACKWARD, Direction.LEFT, true, 560, 2.0, true);
            } else {
                robot.driveTicks(150, 0.9, 0.9, 0.9, 0.9);
            }

            // Turn to drive head on into cube
            robot.turn(-90, 1.5);

            if (getStartingPosition() == StartingPosition.DEPOT) {
                // Hit cube
                if (goldMineral == MineralPosition.LEFT) {
//                    robot.driveTicks(1650, 0.9, -0.9, -0.9, 0.9);
                    robot.driveTicks(1650, 1, 1, 1, 1);
                    //robot.aligner.driveToDistance(Direction.RIGHT, Direction.LEFT, true, 100, 2.0, true);
                } else if (goldMineral == MineralPosition.RIGHT) {
//                    robot.driveTicks(1650, 0.9, -0.9, -0.9, 0.9);
                    robot.driveTicks(1650, 1, 1, 1, 1);
                    //robot.aligner.driveToDistance(Direction.RIGHT, Direction.RIGHT, true, 100, 2.0, true);
                } else {
                    robot.driveTicks(1650, 1, 1, 1, 1);
//                    robot.aligner.driveToDistance(Direction.RIGHT, Direction.LEFT, true, 200, 1.5, true);
                }

                robot.turn(0, 1.5);

                // Recenter
                if (goldMineral == MineralPosition.LEFT) {
                    robot.driveTicks(-MINERAL_TICKS, -0.9, -0.9, -0.9, -0.9);
//                    robot.driveTicks(100 - MINERAL_TICKS - 100, -0.9, -0.9, -0.9, -0.9);
//                    robot.driveTicks(100 - MINERAL_TICKS, -0.9, -0.9, -0.9, -0.9);
                } else if (goldMineral == MineralPosition.RIGHT) {
                    robot.driveTicks(MINERAL_TICKS, 0.9, 0.9, 0.9, 0.9);
//                    robot.driveTicks(100 + MINERAL_TICKS, 0.9, 0.9, 0.9, 0.9);
//                    robot.driveTicks(100 + MINERAL_TICKS, 0.9, 0.9, 0.9, 0.9);
                }

                robot.teamMarker.setPosition(Robot.SERVO_TEAM_MARKER_DEPOSIT);
                robot.setupSimpleServos(Direction.RIGHT);

                sleep(400);

                // Turn and strafe into wall
                if (!shouldEndInOtherCrater()) {
                    robot.turn(-45);
                    robot.driveTicks(600, 0.9, -0.9, -0.9, 0.9);
                } else {
                    robot.turn(45);
                    robot.driveTicks(800, 0.9, -0.9, -0.9, 0.9);
                }
            } else if (getStartingPosition() == StartingPosition.CRATER) {
                // Hit cube
                if (goldMineral == MineralPosition.LEFT) {
                    robot.driveTicks(800, 0.9, 0.9, 0.9, 0.9);
                    robot.driveTicks(-250, -0.9, -0.9, -0.9, -0.9);
//                    robot.driveTicks(800, 0.9, -0.9, -0.9, 0.9);
//                    robot.driveTicks(-650, -0.9, 0.9, 0.9, -0.9);
                } else if (goldMineral == MineralPosition.RIGHT) {
                    robot.driveTicks(800, 0.9, 0.9, 0.9, 0.9);
                    robot.driveTicks(-250, -0.9, -0.9, -0.9, -0.9);
//                    robot.driveTicks(800, 0.9, -0.9, -0.9, 0.9);
//                    robot.driveTicks(-650, -0.9, 0.9, 0.9, -0.9);
                } else {
//                    robot.turn(-90);
                    robot.driveTicks(800, 0.9, 0.9, 0.9, 0.9);
                    robot.driveTicks(-250, -0.9, -0.9, -0.9, -0.9);
//                    robot.turn(0, 1.5);
                }

                robot.turn(0, 2.0);

                // Drive to wall
                if (goldMineral == MineralPosition.LEFT) {
                    robot.driveTicks(2000 - MINERAL_TICKS - 100, 0.9, 0.9, 0.9, 0.9);
                } else if (goldMineral == MineralPosition.RIGHT) {
                    robot.driveTicks(2000 + MINERAL_TICKS, 0.9, 0.9, 0.9, 0.9);
                } else {
                    robot.driveTicks(2000, 0.9, 0.9, 0.9, 0.9);
                }

                robot.setupSimpleServos(Direction.RIGHT);

                // Turn and strafe to wall
                robot.turn(45);

                robot.driveTicks(250, 0.9, -0.9, -0.9, 0.9);

                // Possible wait for alliance to do their thing
//                sleep(4000);

                timer.reset();

                // Drive to depot (ENCODE!!!)
//            robot.aligner.driveAlignDistanceTicks(0.9, 90, 2100, false);
                // IN CASE SENSORS DON'T WORK:
//            robot.driveTicks(2100, 0.9, 0.9, 0.9, 0.9);

                while (opModeIsActive()) {
                    robot.aligner.driveAlignDistance(0.9, 100, false);
                    if (timer.seconds() > 1.05) break;
                    idle();
                }
                robot.driveMotors(0, 0, 0, 0);

                robot.teamMarker.setPosition(Robot.SERVO_TEAM_MARKER_DEPOSIT);
                sleep(500);
            }

            robot.latchLeft.setPower(0.0);
            robot.latchRight.setPower(0.0);

            timer.reset();

            // Head back to crater (ENCODE!!!)
//        if (getStartingPosition() == StartingPosition.CRATER) {
//            robot.aligner.driveAlignDistanceTicks(0.9, 90, 3200, false);
//             IN CASE SENSORS DON'T WORK:
//            robot.driveTicks(3200, 0.9, 0.9, 0.9, 0.9);
//        } else {
//            if (!shouldEndInOtherCrater()) {
//                robot.aligner.driveAlignDistanceTicks(-0.9, 90, -3200, false);
//             IN CASE SENSORS DON'T WORK:
//                robot.driveTicks(-3200, -0.9, -0.9, -0.9, -0.9);
//            } else {
//                robot.aligner.driveAlignDistanceTicks(0.9, 90, 2800, false);
//             IN CASE SENSORS DON'T WORK:
//                robot.driveTicks(2800, 0.9, 0.9, 0.9, 0.9);
//            }
//        }
            while (opModeIsActive()) {
                if (!shouldEndInOtherCrater()) {
                    robot.aligner.driveAlignDistance(-0.9, 100, false);
                } else {
                    robot.aligner.driveAlignDistance(0.9, 100, false);
                }
                if (getStartingPosition() == StartingPosition.CRATER) {
                    if (timer.seconds() > 0.95) break;
                } else {
                    if (timer.seconds() > 0.7) break;
                }

                idle();
            }

            // Turn and unfurl arm
            if (getStartingPosition() == StartingPosition.DEPOT) {
                if (!shouldEndInOtherCrater()) {
                    robot.turn(135, 2.5);
                }
            } else {
                robot.turn(-135, 2.5);
            }

            if (superTimer.seconds() <= 25) {
                robot.lenny.setPower(1.0);
                sleep(3000);
                robot.lenny.setPower(0.0);
            }

            // Save heading
            PersistentHeading.saveHeading(robot.getHeading());
        }
    }
}
