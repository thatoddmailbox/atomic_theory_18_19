package org.firstinspires.ftc.teamcode.opmodes.auto;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.PIDCoefficients;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.robot.Robot;
import org.firstinspires.ftc.teamcode.utils.PIDController;

public class AutoAligner {
    LinearOpMode opmode;

    public AutoAligner(LinearOpMode opmode) {
        this.opmode = opmode;
    }

    //       ***THE ROBOT***
    //
    // HALF       ZERO        HALF
    //
    //          L      R
    //
    //       R  FL--F--FR  L
    //          |      |
    // FULL     L      R      FULL
    //          |      |
    //       L  BL--B--BR  R
    //
    //          R      L
    //
    // HALF       ZERO        HALF
    //

    // Aligns robot on wall by turning in place (called once)
    public void alignRobot(Robot robot, Robot.Direction direction, boolean shouldLog) throws InterruptedException {
        robot.setupSimpleServos(direction);

        double leftDistance = robot.leftDistance(direction);
        double rightDistance = robot.rightDistance(direction);
        double distanceDiff = leftDistance-rightDistance;

        while ((leftDistance + 2 < rightDistance || leftDistance - 2 > rightDistance) && opmode.opModeIsActive()) {
            if (shouldLog) {
                robot.logSensors();
            }

            leftDistance = robot.leftDistance(direction);
            rightDistance = robot.rightDistance(direction);

            if (rightDistance == 2550 || leftDistance == 2550) {
                continue;
            }

            int distanceSign = distanceDiff > 0 ? 1 : -1;
            double impulsePower = distanceSign * (0.4 + Math.abs(distanceDiff/500));

            robot.driveMotors(-1*impulsePower, -1*impulsePower, impulsePower, impulsePower);

            opmode.idle();
        }
    }

    public void pidAlignRobot(Robot robot) {
        double leftDistance = robot.rangeFrontRight.cmUltrasonic() * 10;
        double rightDistance = robot.rangeBackRight.cmUltrasonic() * 10;
        double distanceDiff = leftDistance-rightDistance;
        while (rightDistance == 2550 || leftDistance == 2550) {
            leftDistance = robot.rangeFrontRight.cmUltrasonic() * 10;
            rightDistance = robot.rangeBackRight.cmUltrasonic() * 10;
            distanceDiff = leftDistance-rightDistance;
        }
        int distanceSign = distanceDiff > 0 ? 1 : -1;
        distanceDiff = distanceSign * distanceDiff;
        robot.lessBadTurn(robot.getHeading() + distanceSign*Math.atan(distanceDiff/150));
    }

    // Drives robot forward/backward while aligning on the wall (called in a loop)
    public void driveAlignRobot(Robot robot, double motorPower) {
        double leftDistance = robot.rangeFrontRight.cmUltrasonic() * 10;
        double rightDistance = robot.rangeBackRight.cmUltrasonic() * 10;
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

    public void pidCornerCenterRobot(Robot robot, double timeout, boolean shouldLog) throws InterruptedException {
        robot.frontRightServo.setPosition(Robot.SENSOR_SERVO_HALF);
        robot.backRightServo.setPosition(Robot.SENSOR_SERVO_HALF);

        Thread.sleep(110);

        double leftDistance;
        double rightDistance;
        double distanceDiff;

        PIDController pid = new PIDController(new PIDCoefficients(0.005, 0.000005, 0.75), true, 0.8);

        PIDController anglePID = new PIDController(new PIDCoefficients(0.0064, 0.00001, 0.072), true, 0.2);
        double targetHeading = robot.getHeading();

        ElapsedTime timer = new ElapsedTime();
        int correctFrames = 0;

        timer.reset();

        while (timer.seconds() < timeout && opmode.opModeIsActive()) {
            if (shouldLog) {
                robot.logSensors();
            }

            leftDistance = robot.rangeFrontRight.cmUltrasonic() * 10;
            rightDistance = robot.rangeBackRight.cmUltrasonic() * 10;

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

            double output = pid.step(-distanceDiff, 0);

            // Angle correction
            double currentHeading = robot.getHeading();

            if (Math.abs(currentHeading - targetHeading) < 0.25) {
                currentHeading = targetHeading;
            }

            double angleCorrection = anglePID.step(currentHeading, targetHeading);

            robot.driveMotors(output - angleCorrection, output + angleCorrection, output - angleCorrection, output + angleCorrection);

            opmode.idle();
        }
    }

    // Drives robot forward/backward while aligning on the wall and maintaining a target distance (mm) (called in a loop)
    public void driveAlignDistanceRobot(Robot robot, double motorPower, double targetDistance) {
        double leftDistance = robot.rangeFrontRight.cmUltrasonic() * 10;
        double rightDistance = robot.rangeBackRight.cmUltrasonic() * 10;
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

    public void driveToDistance(Robot robot, Robot.Direction direction, double targetDistance, double timeout, boolean shouldLog) throws InterruptedException {
        switch (direction) {
            case FORWARD:
                robot.frontRightServo.setPosition(Robot.SENSOR_SERVO_ZERO);
            case RIGHT:
                robot.frontRightServo.setPosition(Robot.SENSOR_SERVO_FULL);
            case LEFT:
                robot.frontLeftServo.setPosition(Robot.SENSOR_SERVO_FULL);
            case BACKWARD:
                robot.frontLeftServo.setPosition(Robot.SENSOR_SERVO_ZERO);

        }
        Thread.sleep(210);

        double leftDistance = robot.leftDistance(direction);

        double distanceDiff =  leftDistance - targetDistance;

        PIDController pid = new PIDController(new PIDCoefficients(0.005, 0.000005, 0.75), true, 0.8);

        PIDController anglePID = new PIDController(new PIDCoefficients(0.0064, 0.00001, 0.072), true, 0.2);
        double targetHeading = robot.getHeading();

        ElapsedTime timer = new ElapsedTime();
        int correctFrames = 0;

        timer.reset();

        while (timer.seconds() < timeout && opmode.opModeIsActive()) {
            if (shouldLog) {
                robot.logSensors();
            }

            leftDistance = robot.leftDistance(direction);
            distanceDiff =  leftDistance - targetDistance;

            if (leftDistance == 2550) {
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

            double output = pid.step(-distanceDiff, 0);

            double frontLeft = (direction == Robot.Direction.FORWARD || direction == Robot.Direction.RIGHT) ? output : -output;
            double frontRight = (direction == Robot.Direction.FORWARD || direction == Robot.Direction.LEFT) ? output : -output;
            double backLeft = (direction == Robot.Direction.FORWARD || direction == Robot.Direction.LEFT) ? output : -output;
            double backRight = (direction == Robot.Direction.FORWARD || direction == Robot.Direction.RIGHT) ? output : - output;

            // Angle correction
            double currentHeading = robot.getHeading();

            if (Math.abs(currentHeading - targetHeading) < 0.25) {
                currentHeading = targetHeading;
            }

            double angleCorrection = anglePID.step(currentHeading, targetHeading);

            robot.driveMotors(frontLeft - angleCorrection, frontRight + angleCorrection, backLeft - angleCorrection, backRight + angleCorrection);

            opmode.idle();
        }
    }

    public void driveAlignDistanceRobotTime(Robot robot, double motorPower, double targetDistance, double time, boolean shouldLog) {
        ElapsedTime elapsedTime = new ElapsedTime();
        while (elapsedTime.milliseconds() < time && opmode.opModeIsActive()) {
            driveAlignDistanceRobot(robot, motorPower, targetDistance);

            if (shouldLog) {
                robot.logSensors();
            }
            opmode.idle();
        }
    }
}
