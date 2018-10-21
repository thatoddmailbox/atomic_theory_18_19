package org.firstinspires.ftc.teamcode.opmodes.teleop;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@TeleOp(name="Mechanum Tele-op")
public class MechanumTele extends LinearOpMode {

    public void setClippedMotorPower(DcMotor motor, double power) {
        if (Math.abs(power) < 0.2) {
            motor.setPower(0);
        } else {
            motor.setPower((power < 0 ? -1 : 1) * Math.min(Math.abs(power), 1));
        }
    }

    @Override
    public void runOpMode() {
        DcMotor frontLeft = hardwareMap.dcMotor.get("front_left");
        DcMotor frontRight = hardwareMap.dcMotor.get("front_right");
        DcMotor backLeft = hardwareMap.dcMotor.get("back_left");
        DcMotor backRight = hardwareMap.dcMotor.get("back_right");

        DcMotor george = hardwareMap.dcMotor.get("george");
        DcMotor lenny = hardwareMap.dcMotor.get("lenny");

        frontLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        frontRight.setDirection(DcMotorSimple.Direction.FORWARD);
        backLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        backRight.setDirection(DcMotorSimple.Direction.FORWARD);

        george.setDirection(DcMotorSimple.Direction.FORWARD);
        lenny.setDirection(DcMotorSimple.Direction.FORWARD);

        frontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        george.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        lenny.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

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
            if(Math.abs(drivePower)>deadZone ||
                    Math.abs(strafePower) > deadZone || Math.abs(turnPower) > deadZone){

                //Sets up variables
                double robotAngle = Math.atan2(drivePower, strafePower) - Math.PI / 4;

                double biggerStick = Math.max(Math.abs(strafePower),Math.abs(drivePower));
                double biggerValue = Math.max(Math.abs(Math.cos(robotAngle)),Math.abs(Math.sin(robotAngle)));

                //Does triggy stuff
                double FL = (biggerStick * Math.cos(robotAngle)/biggerValue * (maxPower-Math.abs(maxPower*turnPower))) + (turnPower*maxPower);
                double FR = (biggerStick * Math.sin(robotAngle)/biggerValue * (maxPower-Math.abs(maxPower*turnPower))) - (turnPower*maxPower);
                double BL = (biggerStick * Math.sin(robotAngle)/biggerValue * (maxPower-Math.abs(maxPower*turnPower))) + (turnPower*maxPower);
                double BR = (biggerStick * Math.cos(robotAngle)/biggerValue * (maxPower-Math.abs(maxPower*turnPower))) - (turnPower*maxPower);

                setClippedMotorPower(frontLeft, FL);
                setClippedMotorPower(frontRight, FR);
                setClippedMotorPower(backLeft, BL);
                setClippedMotorPower(backRight, BR);
            }
            else{
                frontLeft.setPower(0);
                frontRight.setPower(0);
                backLeft.setPower(0);
                backRight.setPower(0);
            }

            //Lenny Control
            if (lennyBackPower > deadZone) {
                lenny.setPower(-lennyBackPower);
            }
            else if(lennyForwardPower > deadZone){
                lenny.setPower(lennyForwardPower);
            }
            else lenny.setPower(0);


            //George Control
            if(gamepad1.x) george.setPower(.9);
            else if(gamepad1.y) george.setPower(-.9);
            else george.setPower(0);

//            //Mechanum Wheel Drive
//            if (Math.abs(drive_power) > dead_zone) {
//                frontLeft.setPower(drive_power * scale);
//                frontRight.setPower(drive_power * scale);
//                backLeft.setPower(drive_power * scale);
//                backRight.setPower(drive_power * scale);
//            } else if (Math.abs(strafe_power) > dead_zone) {
//                frontLeft.setPower(strafe_power * scale);
//                frontRight.setPower(-strafe_power * scale);
//                backLeft.setPower(-strafe_power * scale);
//                backRight.setPower(strafe_power * scale);
//            } else if (Math.abs(turn_power) > dead_zone) {
//                frontLeft.setPower(turn_power * scale);
//                frontRight.setPower(-turn_power * scale);
//                backLeft.setPower(turn_power * scale);
//                backRight.setPower(-turn_power * scale);
//            } else {
//                frontLeft.setPower(0);
//                frontRight.setPower(0);
//                backLeft.setPower(0);
//                backRight.setPower(0);
//            }

        }
    }
}
