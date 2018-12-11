package org.firstinspires.ftc.teamcode.opmodes.tests;

import com.qualcomm.hardware.bosch.BNO055IMU;
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

import java.net.SocketException;
import java.net.UnknownHostException;

@Autonomous(name="Only IMU test", group="Tests")
public class OnlyIMUTest extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        BNO055IMU.Parameters imuParameters = new BNO055IMU.Parameters();

        imuParameters.mode                = BNO055IMU.SensorMode.NDOF;
        imuParameters.angleUnit           = BNO055IMU.AngleUnit.DEGREES;
        imuParameters.accelUnit           = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        imuParameters.loggingEnabled      = false;

        BNO055IMU imu = hardwareMap.get(BNO055IMU.class, "imu");

        imu.initialize(imuParameters);

        waitForStart();

        while (opModeIsActive()) {
            double currentHeading = imu.getAngularOrientation(AxesReference.EXTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle;

            /*
             * telemetry
             */
            telemetry.addData("Current", currentHeading);
            telemetry.update();

            idle();
        }
    }
}
