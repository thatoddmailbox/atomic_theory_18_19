package org.firstinspires.ftc.teamcode.opmodes.tests;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.blackbox.MatchPhase;
import org.firstinspires.ftc.teamcode.opmodes.auto.AutoAligner;
import org.firstinspires.ftc.teamcode.robot.Robot;
import org.firstinspires.ftc.teamcode.utils.RobotFeature;

@Autonomous(name="Servo test", group="Tests")
public class ServoTest extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        try (Robot robot = new Robot(MatchPhase.TEST, this, new RobotFeature[] {})) {
            waitForStart();
            robot.handleMatchStart();

            while (opModeIsActive()) {
                sleep(500);
                robot.frontRightServo.setPosition(Robot.SENSOR_SERVO_ZERO);
                robot.backRightServo.setPosition(Robot.SENSOR_REV_SERVO_ZERO);
                robot.backLeftServo.setPosition(Robot.SENSOR_REV_SERVO_ZERO);
                sleep(1000);
                robot.frontRightServo.setPosition(Robot.SENSOR_SERVO_HALF);
                robot.backRightServo.setPosition(Robot.SENSOR_REV_SERVO_HALF);
                robot.backLeftServo.setPosition(Robot.SENSOR_REV_SERVO_HALF);
                sleep(1000);
                robot.frontRightServo.setPosition(Robot.SENSOR_SERVO_FULL);
                robot.backRightServo.setPosition(Robot.SENSOR_REV_SERVO_FULL);
                robot.backLeftServo.setPosition(Robot.SENSOR_REV_SERVO_FULL);
                sleep(1000);
                robot.frontRightServo.setPosition(Robot.SENSOR_SERVO_HALF);
                robot.backRightServo.setPosition(Robot.SENSOR_REV_SERVO_HALF);
                robot.backLeftServo.setPosition(Robot.SENSOR_REV_SERVO_HALF);
                sleep(1000);
                robot.frontRightServo.setPosition(Robot.SENSOR_SERVO_ZERO);
                robot.backRightServo.setPosition(Robot.SENSOR_REV_SERVO_ZERO);
                robot.backLeftServo.setPosition(Robot.SENSOR_REV_SERVO_ZERO);
                sleep(1000);
                break;
            }
        }
    }
}
