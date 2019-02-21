package org.firstinspires.ftc.teamcode.opmodes.auto;

import com.qualcomm.robotcore.hardware.PIDCoefficients;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.robot.Robot;
import org.firstinspires.ftc.teamcode.utils.Direction;
import org.firstinspires.ftc.teamcode.utils.PIDController;

public class AutoAligner {
    Robot robot;

    public AutoAligner(Robot robot) {
        this.robot = robot;
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
    public void align(Direction direction, boolean shouldLog) throws InterruptedException {
        robot.setupSimpleServos(direction);

        double leftDistance = robot.leftDistance(direction);
        double rightDistance = robot.rightDistance(direction);
        double distanceDiff = leftDistance-rightDistance;

        while ((leftDistance + 2 < rightDistance || leftDistance - 2 > rightDistance) && robot.opMode.opModeIsActive()) {
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

            robot.opMode.idle();
        }
    }

    // Drives robot forward/backward while aligning on the wall (called in a loop)
    public void driveAlign(double motorPower) {
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

    public void centerInCorner(double timeout, boolean shouldLog) throws InterruptedException {
        robot.frontRightServo.setPosition(Robot.SENSOR_SERVO_HALF);
        robot.backRightServo.setPosition(Robot.SENSOR_REV_SERVO_HALF);

        Thread.sleep(110);

        double leftDistance;
        double rightDistance;
        double lastLeftDistance = 0;
        double lastRightDistance = 0;
        double distanceDiff;

        double lastDistanceDiff;

        PIDController pid = new PIDController(new PIDCoefficients(0.002, 0.00000075, 0), true, 0.8);

        PIDController anglePID = new PIDController(new PIDCoefficients(0.0064, 0.00001, 0.072), true, 0.2);
        double targetHeading = robot.getHeading();

        ElapsedTime timer = new ElapsedTime();
        int correctFrames = 0;

        timer.reset();

        double lastCorrectTime = 0;
        double lastLoopTime = -0.09;
        //double errorsInARow = 0;
        while (timer.seconds() < timeout && robot.opMode.opModeIsActive()) {
            if (shouldLog) {
                robot.logSensors();
            }

            leftDistance = robot.rangeFrontRight.cmUltrasonic() * 10;
            rightDistance = robot.rangeBackRight.cmUltrasonic() * 10;

            // Angle correction
            double currentHeading = robot.getHeading();
            if (Math.abs(currentHeading - targetHeading) < 0.25) {
                currentHeading = targetHeading;
            }
            double angleCorrection = anglePID.step(currentHeading, targetHeading);
//            angleCorrection = 0;

            if (leftDistance == 2550 && rightDistance == 2550) {
                startSineWave();
                timeout += timer.seconds() - lastLoopTime;
                lastLoopTime = timer.seconds();
                continue;
            } else if (leftDistance == 2550) {
                if (lastRightDistance == 0) {
                    startSineWave();
                    timeout += timer.seconds() - lastLoopTime;
                    lastLoopTime = timer.seconds();
                    continue;
                }
                double rightDiff = rightDistance - lastRightDistance;
                leftDistance = lastLeftDistance - rightDiff;
            } else if (rightDistance == 2550) {
                if (lastLeftDistance == 0) {
                    startSineWave();
                    timeout += timer.seconds() - lastLoopTime;
                    lastLoopTime = timer.seconds();
                    continue;
                }
                double leftDiff = leftDistance - lastLeftDistance;
                rightDistance = lastRightDistance - leftDiff;
            }

            lastLoopTime = timer.seconds();

            distanceDiff = leftDistance - rightDistance;

            if (Math.abs(distanceDiff) < 20) {
//                correctFrames += 1;
//                if (correctFrames > 20) break;
                if (lastCorrectTime == 0) lastCorrectTime = timer.seconds();
                if (timer.seconds() - lastCorrectTime > 0.3) break;
            } else {
                lastCorrectTime = 0;
                //correctFrames = 0;
            }

            double output = pid.step(-distanceDiff, 0);

            robot.driveMotors(output - angleCorrection, output + angleCorrection, output - angleCorrection, output + angleCorrection);
//            robot.driveMotors(output, output, output, output);

            lastLeftDistance = leftDistance;
            lastRightDistance = rightDistance;

            robot.opMode.idle();
        }
    }

    // Drives robot forward/backward while aligning on the wall and maintaining a target distance (mm) (called in a loop)
    public void driveAlignDistance(double motorPower, double targetWallDistance, boolean followWithSensors) {
        if (followWithSensors) {
            double angle = robot.getHeading();
            double relativeError = angle / 45;
            robot.frontRightServo.setPosition(Robot.SENSOR_SERVO_FULL + (relativeError - 1) * (Robot.SENSOR_SERVO_FULL-Robot.SENSOR_SERVO_HALF));
            robot.backRightServo.setPosition(Robot.SENSOR_REV_SERVO_FULL + (1 - relativeError) * (Robot.SENSOR_REV_SERVO_FULL-Robot.SENSOR_REV_SERVO_HALF));
        }

        double leftDistance = robot.leftDistance(Direction.RIGHT);
        double rightDistance = robot.rightDistance(Direction.RIGHT) + 10;
        // Difference between two sensor readings for wall alignment
        double distanceDiff = leftDistance-rightDistance;
        // Difference between target distance and actual distance to set perpendicular distance
        double distanceError = targetWallDistance - leftDistance;
        if (leftDistance > rightDistance) {
            distanceError = targetWallDistance - rightDistance;
        }

        // Scale to change motor power;
        double correctionToPowerScaling = motorPower/0.5;
        distanceDiff = correctionToPowerScaling * distanceDiff/500;
        distanceError = correctionToPowerScaling * distanceError/500;

        if (rightDistance >= 2550 || leftDistance >= 2550) {
            distanceDiff = 0;
        }

        if (rightDistance >= 2550 && leftDistance >= 2550) {
            distanceError = 0;
        }

        robot.driveMotors(motorPower + Math.signum(motorPower) * (distanceDiff - distanceError), motorPower + Math.signum(motorPower) * (-distanceDiff + distanceError),motorPower + Math.signum(motorPower) * (distanceDiff + distanceError), motorPower + Math.signum(motorPower) * (- distanceDiff - distanceError));
    }

    //NOTE: Supply a negative targetOrthogonalWallDistance to go backwards, and a positive one to go forwards
    public void driveAlignDistance(double targetWallDistance, double targetOrthogonalWallDistance, double timeout, boolean shouldLog) {
        double leftDistance;

        double orthogonalWallDistanceDiff;

        PIDController pid = new PIDController(new PIDCoefficients(0.005, 0.000005, 0.75), true, 0.8);

        ElapsedTime timer = new ElapsedTime();
        int correctFrames = 0;

        timer.reset();

        while (timer.seconds() < timeout && robot.opMode.opModeIsActive()) {
            leftDistance = robot.leftDistance(targetOrthogonalWallDistance > 0 ? Direction.FORWARD : Direction.BACKWARD);
            orthogonalWallDistanceDiff = leftDistance - Math.abs(targetOrthogonalWallDistance);

            if (shouldLog) {
                robot.opMode.telemetry.addData("orthogonal wall distance diff", orthogonalWallDistanceDiff);
                robot.logSensors();
            }

            if (leftDistance == 2550) {
                continue;
            }

            if (Math.abs(orthogonalWallDistanceDiff) < 20) {
                correctFrames += 1;
                if (correctFrames > 20) {
                    break;
                }
            } else {
                correctFrames = 0;
            }

            double output = pid.step(-orthogonalWallDistanceDiff, 0);

            this.driveAlignDistance(Math.signum(targetOrthogonalWallDistance)*output, targetWallDistance, false);

            robot.opMode.idle();
        }

    }

    public void driveToDistance(Direction direction, Direction sensorChoice, boolean half, double targetDistance, double timeout, boolean shouldLog) throws InterruptedException {
        driveToDistance(direction, sensorChoice, half, targetDistance, timeout, shouldLog, false);
    }

    public void startSineWave() {
        ElapsedTime timer = new ElapsedTime();
        while (robot.opMode.opModeIsActive() && timer.seconds() < 0.25) {
            robot.driveMotors(Math.sin(timer.seconds()*8*Math.PI), Math.sin(timer.seconds()*8*Math.PI), Math.sin(timer.seconds()*8*Math.PI), Math.sin(timer.seconds()*8*Math.PI));
        }
    }

    public void driveToDistance(Direction direction, Direction sensorChoice, boolean half, double targetDistance, double timeout, boolean shouldLog, boolean shouldCenter) throws InterruptedException {
        switch (direction) {
            case FORWARD:
                if (!half) {
                    robot.frontRightServo.setPosition(Robot.SENSOR_SERVO_ZERO);
                    robot.backRightServo.setPosition(Robot.SENSOR_SERVO_ZERO);
                    //robot.frontLeftServo.setPosition(Robot.SENSOR_SERVO_ZERO);
                } else {
                    robot.frontRightServo.setPosition(Robot.SENSOR_SERVO_HALF);
                    robot.backRightServo.setPosition(Robot.SENSOR_SERVO_HALF);
                    //robot.frontLeftServo.setPosition(Robot.SENSOR_SERVO_HALF);
                }
                break;
            case RIGHT:
                if (!half) {
                    robot.backRightServo.setPosition(Robot.SENSOR_REV_SERVO_FULL);
                    robot.frontRightServo.setPosition(Robot.SENSOR_SERVO_FULL);
                } else {
                    robot.backRightServo.setPosition(Robot.SENSOR_REV_SERVO_HALF);
                    robot.frontRightServo.setPosition(Robot.SENSOR_SERVO_HALF);
                }
                break;
            case LEFT:
                if (!half) {
                    //robot.frontLeftServo.setPosition(Robot.SENSOR_SERVO_FULL);
                    //robot.backLeftServo.setPosition(Robot.SENSOR_SERVO_FULL);
                } else {
                    //robot.frontLeftServo.setPosition(Robot.SENSOR_SERVO_HALF);
                    //robot.backLeftServo.setPosition(Robot.SENSOR_SERVO_HALF);
                }
                break;
            case BACKWARD:
                if (!half) {
                    //robot.backLeftServo.setPosition(Robot.SENSOR_SERVO_ZERO);
                    robot.backRightServo.setPosition(Robot.SENSOR_REV_SERVO_ZERO);
                } else {
                    //robot.backLeftServo.setPosition(Robot.SENSOR_SERVO_HALF);
                    robot.backRightServo.setPosition(Robot.SENSOR_REV_SERVO_HALF);
                }
                break;
        }
        Thread.sleep(210);

        double distance;

        double leftDistance;
        double rightDistance;
        double lastLeftDistance = 0;
        double lastRightDistance = 0;

        double distanceDiff;

        //0.000004
        //0.0005

        PIDController centerCornerPID = new PIDController(new PIDCoefficients(0.002, 0.00000075, 0), true, 0.6);

        PIDController pid = new PIDController(new PIDCoefficients(0.006, 0.0000015, 0.0), true, 0.8);

        PIDController anglePID = new PIDController(new PIDCoefficients(0.0064, 0.00001, 0.072), true, 0.2);
        double targetHeading = robot.getHeading();

        ElapsedTime timer = new ElapsedTime();
        int correctFrames = 0;

        timer.reset();

        double lastCorrectTime = 0;
        double lastLoopTime = -0.09;
        while (timer.seconds() < timeout && robot.opMode.opModeIsActive()) {
            if (shouldLog) {
                robot.logSensors();
            }

            leftDistance = robot.leftDistance(Direction.RIGHT);
            rightDistance = robot.rightDistance(Direction.RIGHT);

            if (leftDistance == 2550 && rightDistance == 2550) {
                startSineWave();
                timeout += timer.seconds() - lastLoopTime;
                lastLoopTime = timer.seconds();
                continue;
            } else if (leftDistance == 2550) {
                if (lastRightDistance == 0) {
                    startSineWave();
                    timeout += timer.seconds() - lastLoopTime;
                    lastLoopTime = timer.seconds();
                    continue;
                }
                double rightDiff = rightDistance - lastRightDistance;
                leftDistance = lastLeftDistance - rightDiff;
            } else if (rightDistance == 2550) {
                if (lastLeftDistance == 0) {
                    startSineWave();
                    timeout += timer.seconds() - lastLoopTime;
                    lastLoopTime = timer.seconds();
                    continue;
                }
                double leftDiff = leftDistance - lastLeftDistance;
                rightDistance = lastRightDistance - leftDiff;
            }

            if (half) {
                if (direction == Direction.FORWARD) {
                    distance = leftDistance;
                } else if (direction == Direction.RIGHT) {
                    if (sensorChoice == Direction.RIGHT) {
                        distance = rightDistance;
                    } else {
                        distance = leftDistance;
                    }
                } else if (direction == Direction.LEFT) {
                    distance = rightDistance;
                } else {
                    distance = rightDistance;
                }
            } else {
                if (sensorChoice == Direction.RIGHT) {
                    distance = robot.rightDistance(direction);
                } else {
                    distance = robot.rightDistance(direction);
                }
            }


            lastLoopTime = timer.seconds();

            distanceDiff = distance - targetDistance;

            if (Math.abs(distanceDiff) < 20) {
                if (lastCorrectTime == 0) lastCorrectTime = timer.seconds();
                if (timer.seconds() - lastCorrectTime > 0.3) break;
            } else {
                //correctFrames = 0;
                lastCorrectTime = 0;
            }

            double output = pid.step(-distanceDiff, 0);
            double sign = Math.signum(output);

            output = Math.min(Math.min(1.0, timer.seconds()), Math.abs(output)) * sign;

            double frontLeft = (direction == Direction.FORWARD || direction == Direction.RIGHT) ? output : -output;
            double frontRight = (direction == Direction.FORWARD || direction == Direction.LEFT) ? output : -output;
            double backLeft = (direction == Direction.FORWARD || direction == Direction.LEFT) ? output : -output;
            double backRight = (direction == Direction.FORWARD || direction == Direction.RIGHT) ? output : - output;

            // Angle correction
            double currentHeading = robot.getHeading();

            if (Math.abs(currentHeading - targetHeading) < 0.25) {
                currentHeading = targetHeading;
            }

            double centerDistanceDiff = leftDistance - rightDistance;
            double centerCorrection = centerCornerPID.step(-centerDistanceDiff, 0);
            if (!shouldCenter) {
                centerCorrection = 0;
            }

            double angleCorrection = anglePID.step(currentHeading, targetHeading);
            robot.driveMotors(centerCorrection + frontLeft - angleCorrection, centerCorrection + frontRight + angleCorrection, centerCorrection + backLeft - angleCorrection, centerCorrection + backRight + angleCorrection);

            robot.opMode.idle();
        }
    }

    private double getYDistance(double left, double right) {
        return left*Math.sqrt(2) + Math.sqrt(Math.pow(right, 2) - 2*Math.pow(left, 2)) / 2;
    }

    private double getXDistance(double left, double right) {
        return Math.sqrt(Math.pow(right, 2) - 2*Math.pow(left, 2)) / 2;
    }

    public void dynamicOmniPID(double targetLeft, double targetRight) throws InterruptedException {
        robot.frontRightServo.setPosition(Robot.SENSOR_SERVO_HALF);
        robot.backRightServo.setPosition(Robot.SENSOR_REV_SERVO_HALF);

        Thread.sleep(110);

        double timeout = 5.0;

        double leftDistance;
        double rightDistance;
        double lastLeftDistance = 0;
        double lastRightDistance = 0;
        double currentXDistance;
        double currentYDistance;

        double targetYDistance = getYDistance(targetLeft, targetRight);
        double targetXDistance = getXDistance(targetLeft, targetRight);

        PIDController xPID = new PIDController(new PIDCoefficients(0.002, 0, 0), true, 0.8);
        PIDController yPID = new PIDController(new PIDCoefficients(0.005, 0.0000075, 0.5), true, 0.8);

        PIDController anglePID = new PIDController(new PIDCoefficients(0.0064, 0.00001, 0.072), true, 0.2);
        double targetHeading = robot.getHeading();

        ElapsedTime timer = new ElapsedTime();

        timer.reset();

        double lastCorrectTime = 0;
        double lastLoopTime = -0.09;
        //double errorsInARow = 0;
        while (timer.seconds() < timeout && robot.opMode.opModeIsActive()) {
            robot.logSensors();

            leftDistance = robot.rangeFrontRight.cmUltrasonic() * 10;
            rightDistance = robot.rangeBackRight.cmUltrasonic() * 10;

            // Angle correction
            double currentHeading = robot.getHeading();
            if (Math.abs(currentHeading - targetHeading) < 0.25) {
                currentHeading = targetHeading;
            }
            double angleCorrection = anglePID.step(currentHeading, targetHeading);

            if (leftDistance == 2550 && rightDistance == 2550) {
                robot.driveMotors(0, 0,0,0);
                timeout += timer.seconds() - lastLoopTime;
                lastLoopTime = timer.seconds();
                continue;
            } else if (leftDistance == 2550) {
                // Cool interpolation stuffs
                if (lastRightDistance == 0) {
                    robot.driveMotors(0, 0,0,0);
                    timeout += timer.seconds() - lastLoopTime;
                    lastLoopTime = timer.seconds();
                    continue;
                }
                double rightDiff = rightDistance - lastRightDistance;
                leftDistance = lastLeftDistance - rightDiff;
            } else if (rightDistance == 2550) {
                // Cool interpolation stuffs
                if (lastLeftDistance == 0) {
                    robot.driveMotors(0, 0,0,0);
                    timeout += timer.seconds() - lastLoopTime;
                    lastLoopTime = timer.seconds();
                    continue;
                }
                double leftDiff = leftDistance - lastLeftDistance;
                rightDistance = lastRightDistance - leftDiff;
            }

            lastLoopTime = timer.seconds();

            currentXDistance = getXDistance(leftDistance, rightDistance);
            currentYDistance = getYDistance(leftDistance, rightDistance);

            robot.opMode.telemetry.addData("x distance", currentXDistance);
            robot.opMode.telemetry.addData("y distance", currentYDistance);
            robot.opMode.telemetry.update();

            double absoluteError = Math.abs(currentXDistance) + Math.abs(currentYDistance);
            double xProportionalError = Math.abs(currentXDistance) / absoluteError;
            double yProportionalError = Math.abs(currentYDistance) / absoluteError;

            if (absoluteError < 30) {
                if (lastCorrectTime == 0) lastCorrectTime = timer.seconds();
                if (timer.seconds() - lastCorrectTime > 0.3) break;
            } else {
                lastCorrectTime = 0;
            }

            double xOutput = xPID.step(currentXDistance, targetXDistance) * xProportionalError;
            double yOutput = yPID.step(currentYDistance, targetYDistance) * yProportionalError;

            robot.driveMotors(xOutput - angleCorrection - yOutput, xOutput + angleCorrection + yOutput, xOutput - angleCorrection + yOutput, xOutput + angleCorrection - yOutput);

            lastLeftDistance = leftDistance;
            lastRightDistance = rightDistance;

            robot.opMode.idle();
        }
    }

}
