package org.firstinspires.ftc.teamcode.opmodes.auto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.exception.RobotCoreException;
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
        double leftDistance = robot.distanceLeft.getDistance(DistanceUnit.MM);
        double rightDistance = robot.distanceRight.getDistance(DistanceUnit.MM);
        while (leftDistance + 2 < rightDistance || leftDistance - 2 > rightDistance) {
            double distanceDiff = leftDistance-rightDistance;
            int distanceSign = distanceDiff > 0 ? 1 : -1;
            double impulsePower = distanceSign * (0.4 + Math.abs(distanceDiff/500));

            robot.driveMotors(-1*impulsePower, -1*impulsePower, impulsePower, impulsePower);

            leftDistance = robot.distanceLeft.getDistance(DistanceUnit.MM);
            rightDistance = robot.distanceRight.getDistance(DistanceUnit.MM);
        }
    }

    // Drives robot forward/backward while aligning on the wall (called in a loop)
    public void driveAlignRobot(Robot robot, double motorPower) {
        double leftDistance = robot.distanceLeft.getDistance(DistanceUnit.MM);
        double rightDistance = robot.distanceRight.getDistance(DistanceUnit.MM);
        // Difference between two sensor readings for wall alignment
        double distanceDiff = leftDistance-rightDistance;
        // Scale to change motor power;
        distanceDiff = distanceDiff/500;

        robot.driveMotors(motorPower + distanceDiff, motorPower - distanceDiff,motorPower + distanceDiff, motorPower - distanceDiff);
    }

    // Drives robot forward/backward while aligning on a corner (untested) (called once)
    public void driveAlignCornerRobot(Robot robot, double motorPower) {
        double leftDistance = robot.distanceLeft.getDistance(DistanceUnit.MM);
        double rightDistance = robot.distanceRight.getDistance(DistanceUnit.MM);
        while (leftDistance + 2 < rightDistance || leftDistance - 2 > rightDistance) {
            double distanceDiff = leftDistance-rightDistance;
            int distanceSign = distanceDiff > 0 ? 1 : -1;
            double impulsePower = distanceSign * (0.4 + Math.abs(distanceDiff/500));

            robot.driveMotors(impulsePower, impulsePower, impulsePower, impulsePower);

            leftDistance = robot.distanceLeft.getDistance(DistanceUnit.MM);
            rightDistance = robot.distanceRight.getDistance(DistanceUnit.MM);
        }
    }

    // Drives robot forward/backward while aligning on the wall and maintaining a target distance (mm) (called in a loop)
    public void driveAlignDistanceRobot(Robot robot, double motorPower, double targetDistance) {
        double leftDistance = robot.distanceLeft.getDistance(DistanceUnit.MM);
        double rightDistance = robot.distanceRight.getDistance(DistanceUnit.MM);
        // Difference between two sensor readings for wall alignment
        double distanceDiff = leftDistance-rightDistance;
        // Difference between target distance and actual distance to set perpendicular distance
        double distanceError = targetDistance - leftDistance;
        if (leftDistance > rightDistance) {
            distanceError = targetDistance - rightDistance;
        }

        // Scale to change motor power;
        distanceDiff = distanceDiff/500;
        distanceError = distanceError/500;

        robot.driveMotors(motorPower + distanceDiff - distanceError, motorPower - distanceDiff + distanceError,motorPower + distanceDiff + distanceError, motorPower - distanceDiff - distanceError);
    }

}
