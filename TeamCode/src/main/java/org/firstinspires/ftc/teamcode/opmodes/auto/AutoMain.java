package org.firstinspires.ftc.teamcode.opmodes.auto;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

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
        Robot robot = new Robot(this);

        telemetry.addData("Status", "Ready to go");
        telemetry.addData("Alliance", getCurrentAlliance());
        telemetry.addData("Starting position", getStartingPosition());
        telemetry.update();

        waitForStart();

        telemetry.addData("Status", "Running");
        telemetry.update();

        robot.latch.setPower(-1);
        sleep(8000);
        robot.latch.setPower(0);

        robot.driveMotors(1, -1, -1, 1);
        sleep(750);
        robot.driveMotors(0, 0, 0, 0);

        while (opModeIsActive()) {
            telemetry.update();
            idle();
        }
    }
}
