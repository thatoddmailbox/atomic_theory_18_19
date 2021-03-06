package org.firstinspires.ftc.teamcode.opmodes.tests;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.PIDCoefficients;

import org.firstinspires.ftc.teamcode.blackbox.MatchPhase;
import org.firstinspires.ftc.teamcode.robot.Robot;
import org.firstinspires.ftc.teamcode.utils.PIDController;
import org.firstinspires.ftc.teamcode.utils.RobotFeature;

@Autonomous(name="Marker test", group="Tests")
public class MarkerTest extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        try (Robot robot = new Robot(MatchPhase.TEST, this, new RobotFeature[] {})) {
            waitForStart();
            robot.handleMatchStart();

            while (opModeIsActive()) {

                if (gamepad1.dpad_up) {
                    robot.teamMarker.setPosition(1);
                } else if (gamepad1.dpad_down) {
                    robot.teamMarker.setPosition(Robot.SERVO_TEAM_MARKER_DEPOSIT);
                } else {
                    robot.teamMarker.setPosition(Robot.SERVO_TEAM_MARKER_HELD);
                }
            }
        }
    }
}
