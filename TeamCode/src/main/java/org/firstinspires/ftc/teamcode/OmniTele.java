package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@TeleOp(name="Omnibot Tele-op")
public class OmniTele extends LinearOpMode {

    @Override
    public void runOpMode() {
        DcMotor frontLeft = hardwareMap.dcMotor.get("front_left");
        DcMotor frontRight = hardwareMap.dcMotor.get("front_right");
        DcMotor backLeft = hardwareMap.dcMotor.get("back_left");
        DcMotor backRight = hardwareMap.dcMotor.get("back_right");

        DcMotor nom = hardwareMap.dcMotor.get("nom");
        DcMotor shoulder = hardwareMap.dcMotor.get("shoulder");
        DcMotor elbow = hardwareMap.dcMotor.get("elbow");

        frontLeft.setDirection(DcMotorSimple.Direction.FORWARD);
        frontRight.setDirection(DcMotorSimple.Direction.REVERSE);
        backLeft.setDirection(DcMotorSimple.Direction.FORWARD);
        backRight.setDirection(DcMotorSimple.Direction.REVERSE);

        frontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        frontLeft.setDirection(DcMotorSimple.Direction.FORWARD);
        frontRight.setDirection(DcMotorSimple.Direction.FORWARD);
        backLeft.setDirection(DcMotorSimple.Direction.FORWARD);

        nom.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        shoulder.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        elbow.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);



        telemetry.addData("Status", "Ready to go");
        telemetry.update();

        waitForStart();

        telemetry.addData("Status", "Running");
        telemetry.update();

        while (opModeIsActive()) {
            double drive_power = -1 * gamepad1.left_stick_y;
            double strafe_power = gamepad1.left_stick_x;
            double turn_power = gamepad1.right_stick_x;

            double scale = 0.3;

            double dead_zone = 0.2;


            //Omni Wheel Drive
            if (Math.abs(drive_power) > dead_zone) {
                frontLeft.setPower(drive_power * scale);
                frontRight.setPower(drive_power * scale);
                backLeft.setPower(drive_power * scale);
                backRight.setPower(drive_power * scale);
            } else if (Math.abs(strafe_power) > dead_zone) {
                frontLeft.setPower(strafe_power * scale);
                backLeft.setPower(-strafe_power * scale);
                frontRight.setPower(-strafe_power * scale);
                backRight.setPower(strafe_power * scale);
            } else if (Math.abs(turn_power) > dead_zone) {
                frontLeft.setPower(turn_power * scale);
                backLeft.setPower(turn_power * scale);
                frontRight.setPower(-turn_power * scale);
                backRight.setPower(-turn_power * scale);
            } else {
                frontLeft.setPower(0);
                backLeft.setPower(0);
                frontRight.setPower(0);
                backRight.setPower(0);
            }

            //Shoulder Control
            if(gamepad1.left_bumper) elbow.setPower(.9);
            else if(gamepad1.right_bumper) elbow.setPower(-.9);
            else elbow.setPower(0);

            //Elbow Control
            if(gamepad1.left_trigger>0.1) shoulder.setPower(gamepad1.left_trigger);
            else if(gamepad1.right_trigger>0.1) shoulder.setPower(-gamepad1.right_trigger);
            else shoulder.setPower(0);

            //Nom Control
            if(gamepad1.x) nom.setPower(.9);
            else if(gamepad1.y) nom.setPower(-.9);
            else nom.setPower(0);

        }
    }
}
