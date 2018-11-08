package org.firstinspires.ftc.teamcode.opmodes.tests;

import com.qualcomm.hardware.rev.Rev2mDistanceSensor;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.PIDCoefficients;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.utils.PIDController;

@Autonomous(name="Distance test")
public class DistanceTest extends LinearOpMode {

    Rev2mDistanceSensor distanceSensor;

    @Override
    public void runOpMode() {
        waitForStart();

        distanceSensor = hardwareMap.get(Rev2mDistanceSensor.class, "distance");

        while (opModeIsActive()) {
            telemetry.addData("distance", distanceSensor.getDistance(DistanceUnit.MM));
            telemetry.update();
        }
    }
}
