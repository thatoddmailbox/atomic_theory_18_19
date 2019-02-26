package org.firstinspires.ftc.teamcode.opmodes.tests;

import com.qualcomm.hardware.lynx.LynxNackException;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DigitalChannel;

import org.firstinspires.ftc.teamcode.blackbox.MatchPhase;
import org.firstinspires.ftc.teamcode.hardware.UltrasonicHub;
import org.firstinspires.ftc.teamcode.robot.Robot;
import org.firstinspires.ftc.teamcode.utils.RobotFeature;

@Autonomous(name="Mux test", group="Tests")
public class MuxTest extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {
        try (Robot robot = new Robot(MatchPhase.TEST, this, new RobotFeature[] {})) {
            waitForStart();
            robot.handleMatchStart();

            DigitalChannel muxReset = hardwareMap.digitalChannel.get("mux_reset");
            UltrasonicHub hub = new UltrasonicHub(hardwareMap.appContext, robot.expansionHub2.getRawHub(), 1, muxReset);

            hub.reset();

            while (opModeIsActive()) {
                try {
                    telemetry.addData("Reading", hub.getReadingFromSensor(0));
                } catch (LynxNackException e) {
                    telemetry.addData("Reading", "error");
                    e.printStackTrace();
                }
                telemetry.update();
                idle();
            }
        }
    }
}
