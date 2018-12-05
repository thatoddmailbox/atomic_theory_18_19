package org.firstinspires.ftc.teamcode.opmodes.tests;

import android.util.Log;

import com.kauailabs.navx.ftc.navXPIDController;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.exception.RobotCoreException;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.PIDCoefficients;
import com.qualcomm.robotcore.util.RobotLog;

import org.firstinspires.ftc.teamcode.robot.Robot;
import org.firstinspires.ftc.teamcode.utils.PIDController;
import org.firstinspires.ftc.teamcode.utils.PIDLogger;

import java.net.SocketException;
import java.net.UnknownHostException;
import java.util.Locale;

@Autonomous(name="navX turning test", group="Tests")
public class NavXTurningTest extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        Robot robot = new Robot(this, false);

        waitForStart();

        PIDLogger pidLogger = null;
//        try {
//            pidLogger = new PIDLogger("192.168.49.2");
//        } catch (SocketException | UnknownHostException e) {
//            e.printStackTrace();
//        }

        navXPIDController yawPIDController = null;
        Gamepad lastGamepad = new Gamepad();
        double targetHeading = 0;
        int newTarget = (int) targetHeading;
        int selectedCoefficient = 0;

        double p = 0.05;
        double i = 0.005;
        double d = 0.3;

        waitForStart();

        yawPIDController = new navXPIDController(robot.navX, navXPIDController.navXTimestampedDataSource.YAW);

        /* Configure the PID controller */
        yawPIDController.setSetpoint(targetHeading);
        yawPIDController.setContinuous(true);
        yawPIDController.setOutputRange(-1, 1);
        yawPIDController.setTolerance(navXPIDController.ToleranceType.ABSOLUTE, 2);
        yawPIDController.setPID(p, i, d);
        yawPIDController.enable(true);

        yawPIDController.reset();

        navXPIDController.PIDResult yawPIDResult = new navXPIDController.PIDResult();

        while (opModeIsActive()) {
            /*
             * pid control loop
             */
            double output = 0.0;

            if (yawPIDController.isNewUpdateAvailable(yawPIDResult)) {
                if (yawPIDResult.isOnTarget()) {
                    robot.setDriveMotorZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
                    robot.driveMotors(0, 0, 0, 0);
                } else {
                    output = yawPIDResult.getOutput();
                }
            } else {

            }

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
                    p -= step;
                    if (p < 0) {
                        p = 0;
                    }
                } else if (selectedCoefficient == 1) {
                    i -= step / 10;
                    if (i < 0) {
                        i = 0;
                    }
                } else if (selectedCoefficient == 2) {
                    d -= step;
                    if (d < 0) {
                        d = 0;
                    }
                } else if (selectedCoefficient == 3) {
                    newTarget -= (fineMode ? 1 : 5);
                    yawPIDController.setSetpoint(newTarget);
                }
                yawPIDController.setPID(p, i, d);
            } else if (gamepad1.dpad_right && !lastGamepad.dpad_right) {
                if (selectedCoefficient == 0) {
                    p += step;
                } else if (selectedCoefficient == 1) {
                    i += step / 10;
                } else if (selectedCoefficient == 2) {
                    d += step;
                } else if (selectedCoefficient == 3) {
                    newTarget += (fineMode ? 1 : 5);
                    yawPIDController.setSetpoint(newTarget);
                }
                yawPIDController.setPID(p, i, d);
            }

            if (gamepad1.a && !lastGamepad.a) {
                targetHeading = newTarget;
                yawPIDController.reset();
            }
            if (gamepad1.y && !lastGamepad.y) {
                yawPIDController.reset();
            }

            /*
             * telemetry
             */
            telemetry.addData("New target", newTarget);
            telemetry.addData("Target", targetHeading);
//            telemetry.addData("Current", robot.navX.getYaw());
            telemetry.addData("Output", output);
            telemetry.addData("Total error", yawPIDController.getError());
            telemetry.addData("Coefficients", String.format(Locale.US, "%f, %f, %f", p, i, d));
            telemetry.addData("Selected parameter", "PIDH".charAt(selectedCoefficient));
            telemetry.addData("Gamepad step", step);
            telemetry.update();

//            pidLogger.sendPacket(targetHeading, currentHeading, output);

            try {
                lastGamepad.copy(gamepad1);
            } catch (RobotCoreException e) {
                e.printStackTrace();
            }
        }
    }
}
