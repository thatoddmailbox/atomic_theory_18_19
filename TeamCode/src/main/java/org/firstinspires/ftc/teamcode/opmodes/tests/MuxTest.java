package org.firstinspires.ftc.teamcode.opmodes.tests;

import com.qualcomm.hardware.lynx.LynxNackException;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DigitalChannel;

import org.firstinspires.ftc.teamcode.hardware.UltrasonicHub;
import org.firstinspires.ftc.teamcode.robot.Robot;

@Autonomous(name="Mux test", group="Tests")
public class MuxTest extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {
        waitForStart();

        Robot robot = new Robot(this, false);
        DigitalChannel muxReset = hardwareMap.digitalChannel.get("mux_reset");
        UltrasonicHub hub = new UltrasonicHub(hardwareMap.appContext, robot.expansionHub2, 1, muxReset);

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
