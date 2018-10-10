package org.firstinspires.ftc.teamcode.opmodes.tests;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.PIDCoefficients;

import org.firstinspires.ftc.teamcode.Consts;
import org.firstinspires.ftc.teamcode.utils.PIDController;

@Autonomous(name="Potentiometer test")
public class PotentiometerTest extends LinearOpMode {

    AnalogInput elbowPotentiometer;
    DcMotor elbowMotor;
    PIDController elbowPID;

    static final double ELBOW_POSITION_CLOSED = 0.045;
    static final double ELBOW_POSITION_OPEN = 1.345;

    @Override
    public void runOpMode() {
        waitForStart();

        elbowPotentiometer = hardwareMap.analogInput.get("elbow_pot");
        elbowMotor = hardwareMap.dcMotor.get("elbow");

        elbowMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        elbowPID = new PIDController(new PIDCoefficients(-1, -0.00005, 0));

        while (opModeIsActive()) {
            if (gamepad1.a) {
                moveToPosition_lessCrappy(0.5);
            } else if (gamepad1.y) {
                moveToPosition_crappy(0.5);
            }

            if (gamepad1.left_bumper) {
                elbowMotor.setPower(.3);
            } else if (gamepad1.right_bumper) {
                elbowMotor.setPower(-.3);
            } else {
                elbowMotor.setPower(0);
            }

            telemetry.addData("reading", elbowPotentiometer.getVoltage());
            telemetry.addData("position", getPosition());
            telemetry.update();
        }
    }

    public double getPosition() {
        // 0 is closed, 1 is open
        return ((elbowPotentiometer.getVoltage() - ELBOW_POSITION_CLOSED) / ELBOW_POSITION_OPEN);
    }

    public void moveToPosition_crappy(double target) {
        while (opModeIsActive() && Math.abs(target - getPosition()) > 0.05) {
            double current = getPosition();
            int direction;
            if (target > current) {
                // up
                direction = -1;
            } else {
                // down
                direction = 1;
            }
            elbowMotor.setPower(direction * 0.3);

            telemetry.addData("current", current);
            telemetry.addData("target", target);
            telemetry.update();
        }
    }

    public void moveToPosition_lessCrappy(double target) {
        elbowPID.reset();

        while (opModeIsActive() && !gamepad1.b) {
            double current = getPosition();
            elbowMotor.setPower(elbowPID.step(current, target));

            telemetry.addData("current", current);
            telemetry.addData("target", target);
            telemetry.addData("errorSum", elbowPID.errorSum);
            telemetry.addData("lastDError", elbowPID.lastDError);
            telemetry.update();
        }
    }
}
