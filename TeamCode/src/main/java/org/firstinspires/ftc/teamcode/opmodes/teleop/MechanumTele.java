package org.firstinspires.ftc.teamcode.opmodes.teleop;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.teamcode.robot.Robot;

@TeleOp(name="Mechanum Tele-op")
public class MechanumTele extends LinearOpMode {

    @Override
    public void runOpMode() {
        Robot robot = new Robot(hardwareMap);

        telemetry.addData("Status", "Ready to go");
        telemetry.update();

        waitForStart();

        telemetry.addData("Status", "Running");
        telemetry.update();

        while (opModeIsActive()) {
            double drivePower = -1 * gamepad1.left_stick_y;
            double strafePower = gamepad1.left_stick_x;
            double turnPower = gamepad1.right_stick_x;

            double lennyBackPower = gamepad1.left_trigger;
            double lennyForwardPower = gamepad1.right_trigger;

            double maxPower = 0.9;
            double deadZone = 0.2;

            //Complete Directional Mecanum Driving
            if(Math.abs(drivePower) > deadZone || Math.abs(strafePower) > deadZone || Math.abs(turnPower) > deadZone) {
                //Sets up variables
                double robotAngle = Math.atan2(drivePower, strafePower) - Math.PI / 4;
                double biggerStick = Math.max(Math.abs(turnPower),Math.max(Math.abs(strafePower),Math.abs(drivePower)));
                double biggerDrive = Math.max(Math.abs(strafePower),Math.abs(drivePower));
                double biggerValue = Math.max(Math.abs(Math.cos(robotAngle)),Math.abs(Math.sin(robotAngle)));
                double stickTotal = (biggerDrive+(Math.abs(turnPower)));

                //Does triggy stuff
                double FL = (Math.cos(robotAngle)/biggerValue*biggerDrive/stickTotal) + (turnPower/stickTotal);
                double FR = (Math.sin(robotAngle)/biggerValue*biggerDrive/stickTotal) - (turnPower/stickTotal);
                double BL = (Math.sin(robotAngle)/biggerValue*biggerDrive/stickTotal) + (turnPower/stickTotal);
                double BR = (Math.cos(robotAngle)/biggerValue*biggerDrive/stickTotal) - (turnPower/stickTotal);

                double biggerPower = Math.max(Math.max(Math.abs(FL),Math.abs(FR)),Math.max(Math.abs(BL),Math.abs(FR)));

                robot.driveMotorsClipped(
                        FL*biggerStick/biggerPower*maxPower,
                        FR*biggerStick/biggerPower*maxPower,
                        BL*biggerStick/biggerPower*maxPower,
                        BR*biggerStick/biggerPower*maxPower);
            } else {
                robot.driveMotors(0, 0, 0, 0);
            }

            //Lenny Control
            if (lennyBackPower > deadZone) {
                robot.lenny.setPower(-lennyBackPower);
            } else if (lennyForwardPower > deadZone) {
                robot.lenny.setPower(lennyForwardPower);
            } else {
                robot.lenny.setPower(0);
            }

            //George Control
            if(gamepad1.x) robot.george.setPower(.9);
            else if(gamepad1.y) robot.george.setPower(-.9);
            else robot.george.setPower(0);
        }
    }
}
