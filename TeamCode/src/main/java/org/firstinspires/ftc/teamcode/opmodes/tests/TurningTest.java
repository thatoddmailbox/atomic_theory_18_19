package org.firstinspires.ftc.teamcode.opmodes.tests;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.exception.RobotCoreException;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.PIDCoefficients;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.teamcode.robot.Robot;
import org.firstinspires.ftc.teamcode.utils.PIDController;
import org.firstinspires.ftc.teamcode.utils.PIDLogger;

import java.io.IOException;
import java.net.DatagramPacket;
import java.net.DatagramSocket;
import java.net.InetAddress;
import java.net.SocketException;
import java.net.UnknownHostException;

@Autonomous(name="Turning test", group="Tests")
public class TurningTest extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        Robot robot = new Robot(this);

        waitForStart();

        PIDLogger pidLogger = null;
        try {
            pidLogger = new PIDLogger("192.168.0.28");
        } catch (SocketException | UnknownHostException e) {
            e.printStackTrace();
        }

        PIDController pid = new PIDController(new PIDCoefficients(0.02, 0.01, 0));
        Gamepad lastGamepad = new Gamepad();
        double targetHeading = 0;
        int newTarget = (int) targetHeading;
        int selectedCoefficient = 0;

        pid.reset();

        /*
         *
         *   fl ---- fr
         *   |        |
         *   |        |
         *   |        |
         *   |        |
         *   bl ---- br
         *
         *   positive heading = clockwise
         *   therefore (sp - pv) > 0 => clockwise turn
         *   clockwise turn = fl, bl positive and fr, br negative
         *
         */

        while (opModeIsActive()) {
            /*
             * pid control loop
             */
            double currentHeading = robot.imu.getAngularOrientation(AxesReference.EXTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle;

            double output = pid.step(currentHeading, targetHeading);

            if (Math.abs(output) > 1) {
                output = Math.signum(output);
            }

            robot.driveMotors(output, -output, output, -output);

            /*
             * gamepad input
             */
            boolean fineMode = gamepad1.right_bumper;
            double step = (fineMode ? 0.001 : 0.01);

            if (gamepad1.dpad_up && !lastGamepad.dpad_up) {
                if (selectedCoefficient != 0) {
                    selectedCoefficient--;
                }
            } else if (gamepad1.dpad_down && !lastGamepad.dpad_down) {
                if (selectedCoefficient != 3) {
                    selectedCoefficient++;
                }
            }

            if (gamepad1.dpad_left && !lastGamepad.dpad_left) {
                if (selectedCoefficient == 0) {
                    pid.coefficients.p -= step;
                } else if (selectedCoefficient == 1) {
                    pid.coefficients.i -= step;
                } else if (selectedCoefficient == 2) {
                    pid.coefficients.d -= step;
                } else if (selectedCoefficient == 3) {
                    newTarget -= (fineMode ? 1 : 5);
                }
            } else if (gamepad1.dpad_right && !lastGamepad.dpad_right) {
                if (selectedCoefficient == 0) {
                    pid.coefficients.p += step;
                } else if (selectedCoefficient == 1) {
                    pid.coefficients.i += step;
                } else if (selectedCoefficient == 2) {
                    pid.coefficients.d += step;
                } else if (selectedCoefficient == 3) {
                    newTarget += (fineMode ? 1 : 5);
                }
            }

            if (gamepad1.a && !lastGamepad.a) {
                targetHeading = newTarget;
                pid.reset();
            }

            /*
             * telemetry
             */
            telemetry.addData("New target", newTarget);
            telemetry.addData("Target", targetHeading);
            telemetry.addData("Current", currentHeading);
            telemetry.addData("Output", output);
            telemetry.addData("Total error", pid.errorSum);
            telemetry.addData("Coefficients", pid.coefficients.toString());
            telemetry.addData("Selected parameter", "PIDH".charAt(selectedCoefficient));
            telemetry.addData("Gamepad step", step);
            telemetry.update();

            pidLogger.sendPacket(targetHeading, currentHeading, output);

            try {
                lastGamepad.copy(gamepad1);
            } catch (RobotCoreException e) {
                e.printStackTrace();
            }
        }
    }
}
