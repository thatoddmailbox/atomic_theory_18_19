package org.firstinspires.ftc.teamcode.opmodes.teleop;

import com.qualcomm.hardware.lynx.commands.core.LynxGetBulkInputDataResponse;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.exception.RobotCoreException;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.teamcode.robot.Robot;

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

        int latchLeftStart = robot.latchLeft.getCurrentPosition() - 6000;
        int latchRightStart = robot.latchRight.getCurrentPosition() - 6000;

        int lennyEncoderDown = 0;

        while (opModeIsActive()) {
            double slowMode = gamepad1.left_bumper ? .5 : 1.0;
            boolean latchMode = gamepad1.right_bumper;

            //Square values for finer slow control.
            double drivePower = 0.1572 * Math.pow(6.3594, Math.abs(gamepad1.left_stick_y)) * Math.signum(gamepad1.left_stick_y);
            double strafePower = -1 * 0.1572 * Math.pow(6.3594, Math.abs(gamepad1.left_stick_x)) * Math.signum(gamepad1.left_stick_x);
            double turnPower = .5 * 0.1572 * Math.pow(6.3594, Math.abs(gamepad1.right_stick_x)) * Math.signum(gamepad1.right_stick_x);

            //Swap Driver and turn if latch mode is activated so directions make sense
            if(latchMode){
                double temp = drivePower;
                drivePower = -strafePower;
                strafePower = temp;
            }

            //Lenny Speed
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

            robot.setDriveMotorZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);


            // turn-compensated driving
            if (gamepad1.left_stick_button && !lastGamepad1.left_stick_button) {
                //turnCompensation = !turnCompensation;
            }

            //Lenny Control
            if (lennyBackPower > deadZone) {
                robot.lenny.setPower(-lennyBackPower);
//                robot.lenny.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            } else if (lennyForwardPower > deadZone) { // && (lennyEncoderDown == 0 || robot.lenny.getCurrentPosition() < (lennyEncoderDown - 10))) {
                robot.lenny.setPower(lennyForwardPower);
//                robot.lenny.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
            } else {
                robot.lenny.setPower(0);
            }

            if (gamepad2.b) {
                lennyEncoderDown = robot.lenny.getCurrentPosition();
            } else if (gamepad2.x) {
                lennyEncoderDown = 0;
            }

            //George Control
            if(gamepad2.dpad_up) robot.george.setPower(-1);
            else if(gamepad2.dpad_down) robot.george.setPower(1);
            else robot.george.setPower(0);


            // latch control: press AND HOLD
            if (gamepad2.y) {
                robot.latchLeft.setPower(-0.8);
                robot.latchLeft.setTargetPosition(latchLeftStart);
                robot.latchRight.setPower(-1.0);
                robot.latchRight.setTargetPosition(latchRightStart);

                //Alternative
                //robot.latchLeft.setPower(-0.9);
                //robot.latchRight.setPower(-0.9);
            } else if (gamepad2.a) {
                robot.latchLeft.setPower(0.8);
                robot.latchLeft.setTargetPosition(latchLeftStart+robot.LATCH_DISTANCE);
                robot.latchRight.setPower(1.0);
                robot.latchRight.setTargetPosition(latchRightStart+robot.LATCH_DISTANCE);

                //Alternative
                //robot.latchLeft.setPower(0.9);
                //robot.latchRight.setPower(0.9);

            } else if(gamepad2.dpad_left){
                robot.latchLeft.setPower(-0.8);
                robot.latchLeft.setTargetPosition(robot.latchLeft.getCurrentPosition()-200);

            } else if(gamepad2.dpad_right){
                robot.latchLeft.setPower(+0.8);
                robot.latchLeft.setTargetPosition(robot.latchLeft.getCurrentPosition()+200);

            } else {
                robot.latchLeft.setPower(0);
                robot.latchRight.setPower(0);
            }

            LynxGetBulkInputDataResponse bulkData = robot.getBulkData(robot.expansionHub2);

            telemetry.addData("Zero power", robot.driveMotorZeroPowerBehavior.toString());
            telemetry.addData("Nom power", nomPower);
            telemetry.addData("Latch Left position", robot.latchLeft.getCurrentPosition());
            telemetry.addData("Latch Right position", robot.latchRight.getCurrentPosition());
            telemetry.addData("Turn compensation", turnCompensation);
            telemetry.addData("Heading offset", robot.headingOffset);
            telemetry.addData("Heading", robot.getHeading());
            telemetry.addData("Lenny encoder", robot.lenny.getCurrentPosition());
            telemetry.addData("Lenny down", lennyEncoderDown);
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
