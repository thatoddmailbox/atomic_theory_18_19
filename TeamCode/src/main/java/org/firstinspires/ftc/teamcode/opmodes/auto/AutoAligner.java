package org.firstinspires.ftc.teamcode.opmodes.auto;

import android.graphics.Path;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.exception.RobotCoreException;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.PIDCoefficients;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.robot.Robot;
import org.firstinspires.ftc.teamcode.utils.PIDController;
import org.firstinspires.ftc.teamcode.utils.PIDLogger;

import java.net.SocketException;
import java.net.UnknownHostException;

public class AutoAligner {

    // Aligns robot on wall by turning in place (called once)
    public void alignRobot(Robot robot) {
        double leftDistance = robot.rangeLeft.cmUltrasonic() * 10;
        double rightDistance = robot.rangeRight.cmUltrasonic() * 10;
        while (leftDistance + 2 < rightDistance || leftDistance - 2 > rightDistance) {
            double distanceDiff = leftDistance-rightDistance;
            int distanceSign = distanceDiff > 0 ? 1 : -1;
            double impulsePower = distanceSign * (0.4 + Math.abs(distanceDiff/500));

            robot.driveMotors(-1*impulsePower, -1*impulsePower, impulsePower, impulsePower);

            leftDistance = robot.rangeLeft.cmUltrasonic() * 10;
            rightDistance = robot.rangeRight.cmUltrasonic() * 10;
        }
    }

    // Drives robot forward/backward while aligning on the wall (called in a loop)
    public void driveAlignRobot(Robot robot, double motorPower) {
        double leftDistance = robot.rangeLeft.cmUltrasonic() * 10;
        double rightDistance = robot.rangeRight.cmUltrasonic() * 10;
        // Difference between two sensor readings for wall alignment
        double distanceDiff = leftDistance-rightDistance;

        // Scale to change motor power
        double correctionToPowerScaling = motorPower/0.5;
        distanceDiff = correctionToPowerScaling * distanceDiff/500;

        if (rightDistance == 2550 || leftDistance == 2550) {
            distanceDiff = 0;
        }

        robot.driveMotors(motorPower + distanceDiff, motorPower - distanceDiff,motorPower + distanceDiff, motorPower - distanceDiff);
    }

    // Drives robot forward/backward while aligning on a corner (untested) (called once)
    public void cornerCenterRobot(Robot robot) {
        double leftDistance = robot.rangeLeft.cmUltrasonic() * 10;
        double rightDistance = robot.rangeRight.cmUltrasonic() * 10;
        while (leftDistance + 2 < rightDistance || leftDistance - 2 > rightDistance) {
            double distanceDiff = leftDistance-rightDistance;
            if (rightDistance == 2550 || leftDistance == 2550) {
                distanceDiff = 0;
            }

            int distanceSign = distanceDiff > 0 ? 1 : -1;

            double impulsePower = distanceSign * (0.4 + Math.abs(distanceDiff/500));

            robot.driveMotors(impulsePower, impulsePower, impulsePower, impulsePower);

            leftDistance = robot.rangeLeft.cmUltrasonic() * 10;
            rightDistance = robot.rangeRight.cmUltrasonic() * 10;
        }
    }

    public void pidCornerCenterRobot(Robot robot) {
        double leftDistance = robot.rangeLeft.cmUltrasonic() * 10;
        double rightDistance = robot.rangeRight.cmUltrasonic() * 10;
        double distanceDiff = leftDistance - rightDistance;

        PIDController pid = new PIDController(new PIDCoefficients(0.05, 0, 0.3), false, 0.5);

        ElapsedTime timer = new ElapsedTime();
        int correctFrames = 0;

        timer.reset();

        while (timer.seconds() < 5) {
            leftDistance = robot.rangeLeft.cmUltrasonic() * 10;
            rightDistance = robot.rangeRight.cmUltrasonic() * 10;
            distanceDiff = leftDistance - rightDistance;

            if (rightDistance == 2550 || leftDistance == 2550) {
                continue;
            }

            if (Math.abs(distanceDiff) < 20) {
                correctFrames += 1;
                if (correctFrames > 20) {
                    break;
                }
            } else {
                correctFrames = 0;
            }

            double output = pid.step(distanceDiff, 0);

            robot.driveMotors(-output, -output, output, output);
        }
    }

    public void pidAlignRobot(Robot robot) {
        double leftDistance = robot.rangeLeft.cmUltrasonic() * 10;
        double rightDistance = robot.rangeRight.cmUltrasonic() * 10;
        double distanceDiff = leftDistance-rightDistance;
        while (rightDistance == 2550 || leftDistance == 2550) {
            distanceDiff = leftDistance-rightDistance;
        }
        int distanceSign = distanceDiff > 0 ? 1 : -1;
        distanceDiff = distanceSign * distanceDiff;
        robot.lessBadTurn(robot.getHeading() + distanceSign*Math.atan(distanceDiff/150));
    }

    // Drives robot forward/backward while aligning on the wall and maintaining a target distance (mm) (called in a loop)
    public void driveAlignDistanceRobot(Robot robot, double motorPower, double targetDistance) {
        double leftDistance = robot.rangeLeft.cmUltrasonic() * 10;
        double rightDistance = robot.rangeRight.cmUltrasonic() * 10;
        // Difference between two sensor readings for wall alignment
        double distanceDiff = leftDistance-rightDistance;
        // Difference between target distance and actual distance to set perpendicular distance
        double distanceError = targetDistance - leftDistance;
        if (leftDistance > rightDistance) {
            distanceError = targetDistance - rightDistance;
        }

        // Scale to change motor power;
        double correctionToPowerScaling = motorPower/0.5;
        distanceDiff = correctionToPowerScaling * distanceDiff/500;
        distanceError = correctionToPowerScaling * distanceError/500;

        if (rightDistance == 2550 || leftDistance == 2550) {
            distanceDiff = 0;
        }

        if (rightDistance == 2550 && leftDistance == 2550) {
            distanceError = 0;
        }

        robot.driveMotors(motorPower + distanceDiff - distanceError, motorPower - distanceDiff + distanceError,motorPower + distanceDiff + distanceError, motorPower - distanceDiff - distanceError);
    }

    public enum Direction {
        FORWARD, BACKWARD, LEFT, RIGHT;
    }

    public void driveToDistance(Robot robot, Direction direction, double targetDistance) {
        double leftDistance = robot.rangeLeft.cmUltrasonic() * 10;

        double distanceError = targetDistance - leftDistance;
        distanceError /= 500;

        if (leftDistance == 2550) {
            distanceError = 0;
        }

        double frontLeft = (direction == Direction.FORWARD || direction == Direction.RIGHT) ? 0.4 - distanceError : -0.4 + distanceError;
        double frontRight = (direction == Direction.FORWARD || direction == Direction.LEFT) ? 0.4 - distanceError : -0.4 + distanceError;
        double backLeft = (direction == Direction.BACKWARD || direction == Direction.LEFT) ? 0.4 - distanceError : -0.4 + distanceError;
        double backRight = (direction == Direction.BACKWARD || direction == Direction.RIGHT) ? 0.4 - distanceError : -0.4 + distanceError;

        robot.driveMotors(frontLeft, frontRight, backLeft, backRight);
    }

    public void driveAlignDistanceRobotTime(Robot robot, double motorPower, double targetDistance, double time) {
        ElapsedTime elapsedTime = new ElapsedTime();
        while (elapsedTime.milliseconds() < time) {
            driveAlignDistanceRobot(robot, motorPower, targetDistance);
        }
    }

}
