package org.firstinspires.ftc.teamcode.opmodes.tests;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.blackbox.MatchPhase;
import org.firstinspires.ftc.teamcode.opmodes.auto.AutoAligner;
import org.firstinspires.ftc.teamcode.robot.Robot;
import org.firstinspires.ftc.teamcode.utils.RobotFeature;

@Autonomous(name="Wall alignment + distance test", group="Tests")
public class WallAlignmentAtDistanceTest extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {
        waitForStart();

        Robot robot = new Robot(MatchPhase.TEST, this, new RobotFeature[] {
                RobotFeature.IMU
        });

        //robot.setupSimpleServos(Robot.Direction.RIGHT);

        double angle = robot.getHeading();
        double relativeError = angle / 45;
        telemetry.addData("relativeError", relativeError);
        robot.frontRightServo.setPosition(Robot.SENSOR_SERVO_FULL + (relativeError - 1) * (Robot.SENSOR_SERVO_FULL-Robot.SENSOR_SERVO_HALF));
        robot.backRightServo.setPosition(Robot.SENSOR_REV_SERVO_FULL + (1 - relativeError) * (Robot.SENSOR_REV_SERVO_FULL-Robot.SENSOR_REV_SERVO_HALF));
        telemetry.addData("front right pos", Robot.SENSOR_SERVO_FULL + (relativeError - 1) * (Robot.SENSOR_SERVO_FULL-Robot.SENSOR_SERVO_HALF));
        telemetry.update();

        sleep(250);

        while (opModeIsActive()) {
            telemetry.addData("range front right", robot.rangeFrontRight.cmUltrasonic() * 10);
            telemetry.addData("range back right", robot.rangeBackRight.cmUltrasonic() * 10);

            telemetry.update();

            robot.aligner.driveAlignDistance(0.85, 300, true);
            idle();
        }
    }
}