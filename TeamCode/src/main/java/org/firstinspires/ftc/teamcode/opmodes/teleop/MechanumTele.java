package org.firstinspires.ftc.teamcode.opmodes.teleop;

import com.qualcomm.hardware.lynx.commands.core.LynxGetBulkInputDataResponse;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.exception.RobotCoreException;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.teamcode.robot.Robot;
import org.firstinspires.ftc.teamcode.utils.PersistentHeading;

@TeleOp(name="Mechanum Tele-op")
public class MechanumTele extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {
        Robot robot = new Robot(this, false);

//        boolean haveSavedHeading = PersistentHeading.haveSavedHeading();
//        if (haveSavedHeading) {
//            robot.headingOffset = PersistentHeading.getSavedHeading();
//        }

        telemetry.addData("Status", "Ready to go");
        telemetry.addData("Heading offset", robot.headingOffset);
        telemetry.update();

        waitForStart();

        telemetry.addData("Status", "Running");
        telemetry.update();

        Gamepad lastGamepad1 = new Gamepad();
        Gamepad lastGamepad2 = new Gamepad();
        double nomPower = 1;
        boolean turnCompensation = false;

        while (opModeIsActive()) {
            double slowMode = gamepad1.left_bumper ? .5 : 1.0;

            //Square values for finer slow control.
            double drivePower = 0.1572 * Math.pow(6.3594, Math.abs(gamepad1.left_stick_y)) * Math.signum(gamepad1.left_stick_y);
            double strafePower = -1 * 0.1572 * Math.pow(6.3594, Math.abs(gamepad1.left_stick_x)) * Math.signum(gamepad1.left_stick_x);
            double turnPower = .5 * 0.1572 * Math.pow(6.3594, Math.abs(gamepad1.right_stick_x)) * Math.signum(gamepad1.right_stick_x);

            //Scaled Lenny speed down
            double lennyBackPower = gamepad2.left_trigger;
            double lennyForwardPower = gamepad2.right_trigger;

            //Controller doesn't report center as exactly 0.
            //This is NOT stall correction, see robot.setClippedMotorPower
            double deadZone = 0.13;

            //Complete Directional Mecanum Driving
            if (Math.abs(gamepad1.left_stick_y) > deadZone || Math.abs(gamepad1.left_stick_x) > deadZone || Math.abs(gamepad1.right_stick_x) > deadZone) {
                //Sets up variables
                double robotAngle = Math.atan2(drivePower, strafePower) - Math.PI / 4;

                telemetry.addData("Robot angle", Math.toDegrees(robotAngle));

                if (turnCompensation) {
                    robotAngle -= Math.toRadians(robot.getHeading());
                }

                telemetry.addData("Robot angle new", Math.toDegrees(robotAngle));

                double biggerStick = Math.max(Math.abs(turnPower), Math.max(Math.abs(strafePower), Math.abs(drivePower)));
                double biggerDrive = Math.max(Math.abs(strafePower), Math.abs(drivePower));
                double biggerValue = Math.max(Math.abs(Math.cos(robotAngle)), Math.abs(Math.sin(robotAngle)));
                double stickMax = biggerDrive + Math.abs(turnPower);

                //Does triggy stuff
                double FL = biggerStick * ((Math.cos(robotAngle) / biggerValue * (biggerDrive / stickMax)) + (turnPower / stickMax));
                double FR = biggerStick * ((Math.sin(robotAngle) / biggerValue * (biggerDrive / stickMax)) - (turnPower / stickMax));
                double BL = biggerStick * ((Math.sin(robotAngle) / biggerValue * (biggerDrive / stickMax)) + (turnPower / stickMax));
                double BR = biggerStick * ((Math.cos(robotAngle) / biggerValue * (biggerDrive / stickMax)) - (turnPower / stickMax));

                //Powers Motors
                robot.driveMotorsClipped(FL * slowMode, FR * slowMode, BL * slowMode, BR * slowMode);
            } else {
                robot.driveMotors(0, 0, 0, 0);
            }

            // zero power behavior toggling
            if (gamepad1.right_bumper && !lastGamepad1.right_bumper) {
                if (robot.driveMotorZeroPowerBehavior == DcMotor.ZeroPowerBehavior.BRAKE) {
                    robot.setDriveMotorZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
                } else {
                    robot.setDriveMotorZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                }
            }

            // turn-compensated driving
            if (gamepad1.left_stick_button && !lastGamepad1.left_stick_button) {
//                turnCompensation = !turnCompensation;
            }

            //Lenny Control
            if (lennyBackPower > deadZone) {
                robot.lenny.setPower(-lennyBackPower);
            } else if (lennyForwardPower > deadZone) {
                robot.lenny.setPower(lennyForwardPower);
            } else {
                robot.lenny.setPower(0);
            }

//            //George Control
//            if(gamepad2.dpad_up) robot.george.setPower(1);
//            else if(gamepad2.dpad_down) robot.george.setPower(-1);
//            else robot.george.setPower(0);

            // nom control
            if (gamepad2.dpad_up) {
                robot.nomLeft.setPosition(Robot.SERVO_VEX_FORWARD);
                robot.nomRight.setPosition(Robot.SERVO_VEX_FORWARD);
            } else if (gamepad2.dpad_down) {
                robot.nomLeft.setPosition(Robot.SERVO_VEX_REVERSE);
                robot.nomRight.setPosition(Robot.SERVO_VEX_REVERSE);
            } else {
                robot.nomLeft.setPosition(Robot.SERVO_VEX_NEUTRAL);
                robot.nomRight.setPosition(Robot.SERVO_VEX_NEUTRAL);
            }

            // nom speed control
//            if (gamepad2.dpad_left && !lastGamepad2.dpad_left) {
//                nomPower -= 0.1;
//                if (nomPower < 0) {
//                    nomPower = 0;
//                }
//            } else if (gamepad2.dpad_right && !lastGamepad2.dpad_right) {
//                nomPower += 0.1;
//                if (nomPower > 1) {
//                    nomPower = 1;
//                }
//            }

            // latch control: press AND HOLD
            if (gamepad2.y) {
                robot.latchLeft.setPower(-0.8);
                robot.latchLeft.setTargetPosition(-10000);
                robot.latchRight.setPower(-1.0);
                robot.latchRight.setTargetPosition(robot.latchLeft.getCurrentPosition());

                //Alternative
                //robot.latchLeft.setPower(-0.9);
                //robot.latchRight.setPower(-0.9);
            } else if (gamepad2.a) {
                robot.latchLeft.setPower(0.8);
                robot.latchLeft.setTargetPosition(0);
                robot.latchRight.setPower(1.0);
                robot.latchRight.setTargetPosition(robot.latchLeft.getCurrentPosition());

                //Alternative
                //robot.latchLeft.setPower(0.9);
                //robot.latchRight.setPower(0.9);
            } // Alternative: else {
                //robot.latchLeft.setPower(0);
                //robot.latchRight.setPower(0);
            //}

            LynxGetBulkInputDataResponse bulkData = robot.getBulkData(robot.expansionHub1);

            telemetry.addData("Zero power", robot.driveMotorZeroPowerBehavior.toString());
            telemetry.addData("Nom power", nomPower);
            telemetry.addData("Latch position", robot.latchLeft.getCurrentPosition());
            telemetry.addData("Turn compensation", turnCompensation);
            telemetry.addData("Heading offset", robot.headingOffset);
            telemetry.addData("Heading", robot.getHeading());
            telemetry.addData("FL position", bulkData.getEncoder(Robot.MOTOR_PORT_FRONT_LEFT));
            telemetry.addData("FR position", bulkData.getEncoder(Robot.MOTOR_PORT_FRONT_RIGHT));
            telemetry.addData("BL position", bulkData.getEncoder(Robot.MOTOR_PORT_BACK_LEFT));
            telemetry.addData("BR position", bulkData.getEncoder(Robot.MOTOR_PORT_BACK_RIGHT));
            telemetry.update();

            try {
                lastGamepad1.copy(gamepad1);
                lastGamepad2.copy(gamepad2);
            } catch (RobotCoreException e) {
                e.printStackTrace();
            }
        }
    }
}
