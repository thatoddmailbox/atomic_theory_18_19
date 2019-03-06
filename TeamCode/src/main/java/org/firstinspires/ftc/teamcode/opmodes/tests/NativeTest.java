//package org.firstinspires.ftc.teamcode.opmodes.tests;
//
//import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
//import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
//
//import io.github.thatoddmailbox.atnative.BuildConfig;
//import io.github.thatoddmailbox.atnative.NativeCore;
//
//@Autonomous(name="Native test", group="Tests")
//public class NativeTest extends LinearOpMode {
//    @Override
//    public void runOpMode() {
//        telemetry.addData("getOKString", NativeCore.getOKString());
//        telemetry.addData("OpenCV version", NativeCore.getOpenCVVersion());
//        telemetry.addData("Native code version", BuildConfig.VERSION_NAME);
//        telemetry.update();
//
//        waitForStart();
//
//        while (opModeIsActive()) {
//            idle();
//        }
//    }
//}
