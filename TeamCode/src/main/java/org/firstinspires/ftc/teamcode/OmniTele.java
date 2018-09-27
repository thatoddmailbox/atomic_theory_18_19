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

        frontLeft.setDirection(DcMotorSimple.Direction.FORWARD);
        frontRight.setDirection(DcMotorSimple.Direction.REVERSE);
        backLeft.setDirection(DcMotorSimple.Direction.FORWARD);
        backRight.setDirection(DcMotorSimple.Direction.REVERSE);

        frontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

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
        }
    }
}
