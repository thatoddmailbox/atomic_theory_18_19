package org.firstinspires.ftc.teamcode.opmodes.auto;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.teamcode.robot.Robot;
import org.firstinspires.ftc.teamcode.utils.Alliance;
import org.firstinspires.ftc.teamcode.utils.PowerSetting;
import org.firstinspires.ftc.teamcode.utils.StartingPosition;

public abstract class AutoMain extends LinearOpMode {

    public abstract Alliance getCurrentAlliance();
    public abstract StartingPosition getStartingPosition();

    @Override
    public void runOpMode() throws InterruptedException {
        Robot robot = new Robot(hardwareMap);

        telemetry.addData("Status", "Ready to go");
        telemetry.addData("Alliance", getCurrentAlliance());
        telemetry.addData("Starting position", getStartingPosition());
        telemetry.update();

        waitForStart();

        telemetry.addData("Status", "Running");
        telemetry.update();

        AutoCorrect corrector = new AutoCorrect();

        while (opModeIsActive()) {
            corrector.setCalibratedPower(new PowerSetting(0.6, 0.6, 0.6, 0.6), robot);
            telemetry.addData("Front Left Power", robot.frontLeft.getPower());
            telemetry.addData("Front Right Power", robot.frontRight.getPower());
            telemetry.addData("Back Left Power", robot.backLeft.getPower());
            telemetry.addData("Back Right Power", robot.backRight.getPower());
            telemetry.addData("Angular Orientation", robot.imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle);
            telemetry.addData("Correction", corrector.correction);

            telemetry.update();

            idle();
        }
    }
}
