package org.firstinspires.ftc.teamcode.opmodes.teleop;

import com.qualcomm.hardware.lynx.commands.core.LynxGetBulkInputDataResponse;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.exception.RobotCoreException;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.PIDCoefficients;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.blackbox.MatchPhase;
import org.firstinspires.ftc.teamcode.robot.Robot;
import org.firstinspires.ftc.teamcode.utils.PIDController;
import org.firstinspires.ftc.teamcode.utils.RobotFeature;

@TeleOp(name="Mechanum Tele-op")
public class MechanumTele extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {
        try (Robot robot = new Robot(MatchPhase.TELEOP, this, new RobotFeature[] {})) {
            telemetry.addData("Status", "Ready to go");
            telemetry.addData("Heading offset", robot.imu.headingOffset);
            telemetry.update();

            waitForStart();
            robot.handleMatchStart();

            telemetry.addData("Status", "Running");
            telemetry.update();

            Gamepad lastGamepad1 = new Gamepad();
            Gamepad lastGamepad2 = new Gamepad();
            double nomPower = 1;
            boolean turnCompensation = false;

            int latchLeftStart = robot.latchLeft.getCurrentPosition(); // - Robot.LATCH_DISTANCE;
            int latchRightStart = robot.latchRight.getCurrentPosition(); // - Robot.LATCH_DISTANCE;
            int lennyEncoderDown = 0;

            ElapsedTime timer = new ElapsedTime();
            timer.reset();

            double lastLoopTime = timer.milliseconds();
            double lastLennyPosition = robot.lenny.getCurrentPosition();

            ElapsedTime lennyTimer = new ElapsedTime();
            lennyTimer.reset();
            boolean lennyPressed = false;
            boolean lennyBackPressed = false;

            PIDController pid = new PIDController("latch turn", new PIDCoefficients(0.016, 0.000025, 0.36), true, 0.5);

            while (opModeIsActive()) {
                double slowMode = gamepad1.left_bumper ? .5 : 1.0;
                boolean latchMode = gamepad1.right_bumper;

                //Square values for finer slow control.
                double drivePower = 0.1572 * Math.pow(6.3594, Math.abs(gamepad1.left_stick_y)) * Math.signum(gamepad1.left_stick_y);
                double strafePower = -1 * 0.1572 * Math.pow(6.3594, Math.abs(gamepad1.left_stick_x)) * Math.signum(gamepad1.left_stick_x);
                double turnPower = .5 * 0.1572 * Math.pow(6.3594, Math.abs(gamepad1.right_stick_x)) * Math.signum(gamepad1.right_stick_x);

                //Swap Driver and turn if latch mode is activated so directions make sense
                if (latchMode) {
                    double temp = drivePower;
                    drivePower = -strafePower;
                    strafePower = temp;
                }

                double currentLennyPosition = robot.lenny.getCurrentPosition();
                double currentTime = timer.milliseconds();
                double loopElapsedTime = currentTime - lastLoopTime;
                double lennyVelocity = (lastLennyPosition - currentLennyPosition) / loopElapsedTime;
                lastLoopTime = currentTime;
                lastLennyPosition = currentLennyPosition;

                //Lenny Speed
                double lennyBackPower = gamepad2.left_trigger;
                double lennyForwardPower = gamepad2.right_trigger;

                //Controller doesn't report center as exactly 0.
                //This is NOT stall correction, see robot.setClippedMotorPower
                double deadZone = 0.13;

                //Complete Directional Mecanum Driving
                if (Math.abs(gamepad1.left_stick_y) > deadZone || Math.abs(gamepad1.left_stick_x) > deadZone || Math.abs(gamepad1.right_stick_x) > deadZone) {
                    //Sets up variables
                    double robotAngle = Math.atan2(drivePower, strafePower) - Math.PI / 4;

                    telemetry.addData("Robot angle", Math.toDegrees(robotAngle));

                    if (turnCompensation) {
                        robotAngle -= Math.toRadians(robot.getHeading());
                    }

                    telemetry.addData("Robot angle new", Math.toDegrees(robotAngle));

                    double biggerStick = Math.max(Math.abs(turnPower), Math.max(Math.abs(strafePower), Math.abs(drivePower)));
                    double biggerDrive = Math.max(Math.abs(strafePower), Math.abs(drivePower));
                    double biggerValue = Math.max(Math.abs(Math.cos(robotAngle)), Math.abs(Math.sin(robotAngle)));
                    double stickMax = biggerDrive + Math.abs(turnPower);

                    //Does triggy stuff
                    double FL = biggerStick * ((Math.cos(robotAngle) / biggerValue * (biggerDrive / stickMax)) + (turnPower / stickMax));
                    double FR = biggerStick * ((Math.sin(robotAngle) / biggerValue * (biggerDrive / stickMax)) - (turnPower / stickMax));
                    double BL = biggerStick * ((Math.sin(robotAngle) / biggerValue * (biggerDrive / stickMax)) + (turnPower / stickMax));
                    double BR = biggerStick * ((Math.cos(robotAngle) / biggerValue * (biggerDrive / stickMax)) - (turnPower / stickMax));

                    // Change the two falses to activate wacky mode
                    if (latchMode && false) {
                        if (Math.abs(FL-BR) < Math.abs(FL-FR)) {
                            FL = Math.signum(FL)*Math.max(Math.abs(FL), Math.abs(BR));
                            BR = Math.signum(FL)*Math.max(Math.abs(FL), Math.abs(BR));
                        } else {
                            FL = Math.signum(FL)*Math.max(Math.abs(FL), Math.abs(FR));
                            FR = Math.signum(FL)*Math.max(Math.abs(FL), Math.abs(FR));
                        }
                        if (Math.abs(BL-FR) < Math.abs(BL-BR)) {
                            BL = Math.signum(BL)*Math.max(Math.abs(BL), Math.abs(FR));
                            FR = Math.signum(BL)*Math.max(Math.abs(BL), Math.abs(FR));
                        } else {
                            BL = Math.signum(BL)*Math.max(Math.abs(BL), Math.abs(BR));
                            BR = Math.signum(BL)*Math.max(Math.abs(BL), Math.abs(BR));
                        }
                    }
                    double currentHeading = robot.getHeading();

                    double output = false ? pid.step(currentHeading, 0) : 0;

                    //Powers Motors
                    robot.driveMotorsClipped(FL * slowMode - output, FR * slowMode + output, BL * slowMode - output, BR * slowMode + output);
                } else {
                    robot.driveMotors(0, 0, 0, 0);
                }

                robot.setDriveMotorZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);


                // turn-compensated driving
                if (gamepad1.left_stick_button && !lastGamepad1.left_stick_button) {
                    //turnCompensation = !turnCompensation;
                }

                //Lenny Control
                //&& lennyVelocity < robot.MAX_LENNY_RETRO_VELOCITY
                if (lennyBackPower > deadZone) {
                    if (!lennyBackPressed || !lennyPressed) lennyTimer.reset();
                    lennyPressed = true;
                    lennyBackPressed = true;
//                      .  .
//                   .        .
//                .              .
//              .                  .                  .
//                                   .              .
//                                      .        .
//                                         .  .
                    robot.lenny.setPower(-lennyBackPower * Math.sin(Math.min(lennyTimer.seconds() * Math.PI, Math.PI / 2)));
//                robot.lenny.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                    //&& lennyVelocity > -robot.MAX_LENNY_RETRO_VELOCITY
                } else if (lennyForwardPower > deadZone) { // && (lennyEncoderDown == 0 || robot.lenny.getCurrentPosition() < (lennyEncoderDown - 10))) {
                    if (lennyBackPressed || !lennyPressed) lennyTimer.reset();
                    lennyPressed = true;
                    lennyBackPressed = false;
                    robot.lenny.setPower(lennyForwardPower * Math.sin(Math.min(lennyTimer.seconds() * Math.PI, Math.PI / 2)));
//                robot.lenny.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
                } else {
                    lennyPressed = false;
                    robot.lenny.setPower(0);
                }

                if (gamepad2.b) {
                    lennyEncoderDown = robot.lenny.getCurrentPosition();
                } else if (gamepad2.x) {
                    lennyEncoderDown = 0;
                }

                //George Control
                if (gamepad2.dpad_up) robot.george.setPower(1);
                else if (gamepad2.dpad_down) robot.george.setPower(-1);
                else robot.george.setPower(0);

                // latch control: press AND HOLD
                if (gamepad2.dpad_left || gamepad2.dpad_right) {
                    if (gamepad2.y) {
                        robot.latchLeft.setTargetPosition(robot.latchLeft.getCurrentPosition() - Robot.LATCH_DISTANCE);
                        robot.latchRight.setTargetPosition(robot.latchRight.getCurrentPosition() - Robot.LATCH_DISTANCE);
                        robot.latchLeft.setPower(-0.8);
                        robot.latchRight.setPower(-1.0);
                    } else if (gamepad2.a) {
                        robot.latchLeft.setTargetPosition(robot.latchLeft.getCurrentPosition() + Robot.LATCH_DISTANCE);
                        robot.latchRight.setTargetPosition(robot.latchRight.getCurrentPosition() + Robot.LATCH_DISTANCE);
                        robot.latchLeft.setPower(0.8);
                        robot.latchRight.setPower(1.0);
                    } else {
                        robot.latchLeft.setTargetPosition(robot.latchLeft.getCurrentPosition());
                        robot.latchRight.setTargetPosition(robot.latchRight.getCurrentPosition());
                        robot.latchLeft.setPower(0);
                        robot.latchRight.setPower(0);
                    }
                } else {
                    if (gamepad2.y) {
                        robot.latchLeft.setPower(-0.8);
                        robot.latchLeft.setTargetPosition(latchLeftStart);
                        robot.latchRight.setPower(-1.0);
                        robot.latchRight.setTargetPosition(latchRightStart);
                    } else if (gamepad2.a) {
                        robot.latchLeft.setPower(0.8);
                        robot.latchLeft.setTargetPosition(latchLeftStart + Robot.LATCH_DISTANCE);
                        robot.latchRight.setPower(1.0);
                        robot.latchRight.setTargetPosition(latchRightStart + Robot.LATCH_DISTANCE);
                    } else {
                        robot.latchLeft.setPower(0);
                        robot.latchRight.setPower(0);
                    }
                }

                LynxGetBulkInputDataResponse bulkData = robot.expansionHub2.getBulkData();

                telemetry.addData("Zero power", robot.driveMotorZeroPowerBehavior.toString());
                telemetry.addData("Nom power", nomPower);
                telemetry.addData("Latch Left position", robot.latchLeft.getCurrentPosition());
                telemetry.addData("Latch Right position", robot.latchRight.getCurrentPosition());
                telemetry.addData("Turn compensation", turnCompensation);
                telemetry.addData("Heading offset", robot.imu.headingOffset);
                telemetry.addData("Heading", robot.getHeading());
                telemetry.addData("Lenny encoder", robot.lenny.getCurrentPosition());
                telemetry.addData("Lenny down", lennyEncoderDown);
                telemetry.addData("Lenny velocity", lennyVelocity);
                telemetry.addData("Loop elapsed time", loopElapsedTime);
                telemetry.addData("FL position", bulkData.getEncoder(Robot.MOTOR_PORT_FRONT_LEFT));
                telemetry.addData("FR position", bulkData.getEncoder(Robot.MOTOR_PORT_FRONT_RIGHT));
                telemetry.addData("BL position", bulkData.getEncoder(Robot.MOTOR_PORT_BACK_LEFT));
                telemetry.addData("BR position", bulkData.getEncoder(Robot.MOTOR_PORT_BACK_RIGHT));
//                telemetry.addData("Hub 1 - battery", robot.readADCFromHub(robot.expansionHub1, LynxGetADCCommand.Channel.BATTERY_MONITOR));
//                telemetry.addData("Hub 1 - five volt", robot.readADCFromHub(robot.expansionHub1, LynxGetADCCommand.Channel.FIVE_VOLT_MONITOR));
//                telemetry.addData("Hub 1 - temp", robot.readADCFromHub(robot.expansionHub1, LynxGetADCCommand.Channel.CONTROLLER_TEMPERATURE));
//                telemetry.addData("Hub 1 - total current", robot.readADCFromHub(robot.expansionHub1, LynxGetADCCommand.Channel.BATTERY_CURRENT));
//                telemetry.addData("Hub 1 - servo current", robot.readADCFromHub(robot.expansionHub1, LynxGetADCCommand.Channel.SERVO_CURRENT));
//                telemetry.addData("Hub 2 - motor 0 current", robot.readADCFromHub(robot.expansionHub2, LynxGetADCCommand.Channel.MOTOR0_CURRENT));
//                telemetry.addData("Hub 2 - motor 1 current", robot.readADCFromHub(robot.expansionHub2, LynxGetADCCommand.Channel.MOTOR1_CURRENT));
//                telemetry.addData("Hub 2 - motor 2 current", robot.readADCFromHub(robot.expansionHub2, LynxGetADCCommand.Channel.MOTOR2_CURRENT));
//                telemetry.addData("Hub 2 - motor 3 current", robot.readADCFromHub(robot.expansionHub2, LynxGetADCCommand.Channel.MOTOR3_CURRENT));
                telemetry.update();

                try {
                    lastGamepad1.copy(gamepad1);
                    lastGamepad2.copy(gamepad2);
                } catch (RobotCoreException e) {
                    e.printStackTrace();
                }
            }
        }
    }
}
