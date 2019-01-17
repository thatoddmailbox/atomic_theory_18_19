package org.firstinspires.ftc.teamcode.robot;

import com.kauailabs.navx.ftc.AHRS;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.hardware.lynx.LynxNackException;
import com.qualcomm.hardware.lynx.commands.core.LynxGetBulkInputDataCommand;
import com.qualcomm.hardware.lynx.commands.core.LynxGetBulkInputDataResponse;
import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cRangeSensor;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DeviceInterfaceModule;
import com.qualcomm.robotcore.hardware.PIDCoefficients;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;
import org.firstinspires.ftc.teamcode.Consts;
import org.firstinspires.ftc.teamcode.utils.MineralPosition;
import org.firstinspires.ftc.teamcode.utils.PIDController;

import java.util.List;

public class Robot {
    public static final double SERVO_TEAM_MARKER_DEPOSIT = 0.0;
    public static final double SERVO_TEAM_MARKER_HELD = 0.6;

    public static final double SENSOR_SERVO_ZERO = 0.25;
    public static final double SENSOR_SERVO_HALF = 0.5;
    public static final double SENSOR_SERVO_FULL = 0.75;

    public static final int MOTOR_PORT_FRONT_LEFT = 0;
    public static final int MOTOR_PORT_FRONT_RIGHT = 1;
    public static final int MOTOR_PORT_BACK_LEFT = 2;
    public static final int MOTOR_PORT_BACK_RIGHT = 3;

    public static final int NAVX_DIM_I2C_PORT = 0;

    /*
     * control modules
     */

    public LynxModule expansionHub1;
    public LynxModule expansionHub2;
    public DeviceInterfaceModule dim;

    /*
     * motors
     */

    public DcMotor frontLeft;
    public DcMotor frontRight;
    public DcMotor backLeft;
    public DcMotor backRight;

    public DcMotor george;
    public DcMotor lenny;
    public DcMotor latchLeft;
    public DcMotor latchRight;

    /*
     * vex motors
     */
    public CRServo nomLeft;
    public CRServo nomRight;

    /*
     * servos
     */

    public Servo teamMarker;

    public Servo frontRightServo;
    public Servo backRightServo;
    public Servo frontLeftServo;
    public Servo backLeftServo;


    /*
     * sensors
     */

    public BNO055IMU imu;

    public WebcamName leftWebcam;
    public WebcamName rightWebcam;

    public AnalogInput sonarLeft;
    public AnalogInput sonarRight;

    public ModernRoboticsI2cRangeSensor rangeFrontRight;
    public ModernRoboticsI2cRangeSensor rangeBackRight;
    public ModernRoboticsI2cRangeSensor rangeFrontLeft;
    public ModernRoboticsI2cRangeSensor rangeBackLeft;

    public AHRS navX;

    public DcMotor.ZeroPowerBehavior driveMotorZeroPowerBehavior;

    public VuforiaLocalizer vuforia;
    public TFObjectDetector tfod;

    public double headingOffset = 0;

    private LinearOpMode _opMode;

    public Robot(LinearOpMode opMode, boolean enableVision) throws InterruptedException {
        _opMode = opMode;

        /*
         * control module initialization
         */
        expansionHub1 = opMode.hardwareMap.get(LynxModule.class, "Expansion Hub 1");
        expansionHub2 = opMode.hardwareMap.get(LynxModule.class, "Expansion Hub 2");
//        dim = opMode.hardwareMap.deviceInterfaceModule.get("dim");

        /*
         * motor initialization
        */
        frontLeft = opMode.hardwareMap.dcMotor.get("front_left");
        frontRight = opMode.hardwareMap.dcMotor.get("front_right");
        backLeft = opMode.hardwareMap.dcMotor.get("back_left");
        backRight = opMode.hardwareMap.dcMotor.get("back_right");

        george = opMode.hardwareMap.dcMotor.get("george");
        lenny = opMode.hardwareMap.dcMotor.get("lenny");
        latchLeft = opMode.hardwareMap.dcMotor.get("latch_left");
        latchRight = opMode.hardwareMap.dcMotor.get("latch_right");

        /*
         * vex motor/servo initialization
         */

        //nomLeft = opMode.hardwareMap.crservo.get("nom_left");
        //nomRight = opMode.hardwareMap.crservo.get("nom_right");

        teamMarker = opMode.hardwareMap.servo.get("team_marker");

        frontRightServo = opMode.hardwareMap.servo.get("front_right_servo");
        backRightServo = opMode.hardwareMap.servo.get("back_right_servo");
//        frontLeftServo = opMode.hardwareMap.servo.get("front_left_servo");
//        backLeftServo = opMode.hardwareMap.servo.get("back_left_servo");

        /*
         * motor setup
         */
        frontLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        frontRight.setDirection(DcMotorSimple.Direction.FORWARD);
        backLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        backRight.setDirection(DcMotorSimple.Direction.FORWARD);

        george.setDirection(DcMotorSimple.Direction.FORWARD);
        lenny.setDirection(DcMotorSimple.Direction.FORWARD);
        latchLeft.setDirection(DcMotorSimple.Direction.FORWARD);
        latchRight.setDirection(DcMotorSimple.Direction.FORWARD);

        setDriveMotorZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        george.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        lenny.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        latchLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        latchRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        latchLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        latchRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        // Alternative to consider: based on velocity
        //latchLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        //latchRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);


        /*
         * sensors
         */
        BNO055IMU.Parameters imuParameters = new BNO055IMU.Parameters();

        imuParameters.mode                = BNO055IMU.SensorMode.NDOF;
        imuParameters.angleUnit           = BNO055IMU.AngleUnit.DEGREES;
        imuParameters.accelUnit           = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        imuParameters.loggingEnabled      = false;

        imu = opMode.hardwareMap.get(BNO055IMU.class, "imu");

        imu.initialize(imuParameters);

        leftWebcam = opMode.hardwareMap.get(WebcamName.class, "left_webcam");
        rightWebcam = opMode.hardwareMap.get(WebcamName.class, "right_webcam");

        sonarLeft = opMode.hardwareMap.get(AnalogInput.class, "sonar_left");
        sonarRight = opMode.hardwareMap.get(AnalogInput.class, "sonar_right");

        rangeFrontRight = opMode.hardwareMap.get(ModernRoboticsI2cRangeSensor.class, "range_left");
        rangeBackRight = opMode.hardwareMap.get(ModernRoboticsI2cRangeSensor.class, "range_right");
        //rangeFrontLeft = opMode.hardwareMap.get(ModernRoboticsI2cRangeSensor.class, "range_front_left");
        //rangeBackLeft = opMode.hardwareMap.get(ModernRoboticsI2cRangeSensor.class, "range_back_left");

//        navX = AHRS.getInstance(dim, NAVX_DIM_I2C_PORT, AHRS.DeviceDataType.kProcessedData, (byte)50);
//        navX.zeroYaw();

        /*
         * initialize sensors
         */
        if (enableVision) {
            VuforiaLocalizer.Parameters vuforiaParameters = new VuforiaLocalizer.Parameters();
            vuforiaParameters.vuforiaLicenseKey = Consts.VUFORIA_KEY;
            vuforiaParameters.cameraName = leftWebcam;
            vuforia = ClassFactory.getInstance().createVuforia(vuforiaParameters);

            int tfodMonitorViewId = opMode.hardwareMap.appContext.getResources().getIdentifier("tfodMonitorViewId", "id", opMode.hardwareMap.appContext.getPackageName());
            TFObjectDetector.Parameters tfodParameters = new TFObjectDetector.Parameters(tfodMonitorViewId);
            tfod = ClassFactory.getInstance().createTFObjectDetector(tfodParameters, vuforia);
            tfod.loadModelFromAsset(Consts.TFOD_MODEL_FILE, Consts.TFOD_LABEL_GOLD, Consts.TFOD_LABEL_SILVER);
        }

//        while (navX.isCalibrating() && opMode.opModeIsActive()) {
//            opMode.telemetry.addLine("Calibrating navX...");
//            opMode.telemetry.update();
//
//            Thread.sleep(100);
//            opMode.idle();
//        }
//
//        opMode.telemetry.addLine("Calibrated navX");
//        opMode.telemetry.update();
    }

    /*
     * drive functions
     */
    public void setDriveMotorZeroPowerBehavior(DcMotor.ZeroPowerBehavior newBehavior) {
        driveMotorZeroPowerBehavior = newBehavior;

        frontLeft.setZeroPowerBehavior(newBehavior);
        frontRight.setZeroPowerBehavior(newBehavior);
        backLeft.setZeroPowerBehavior(newBehavior);
        backRight.setZeroPowerBehavior(newBehavior);
    }

    public void setClippedMotorPower(DcMotor motor, double power) {
        if (Math.abs(power) < 0.2) {
            motor.setPower(0);
        } else {
            motor.setPower((power < 0 ? -1 : 1) * Math.min(Math.abs(power), 1));
        }
    }

    public void driveMotors(double fl, double fr, double bl, double br) {
        frontLeft.setPower(fl);
        frontRight.setPower(fr);
        backLeft.setPower(bl);
        backRight.setPower(br);
    }

    public void driveMotorsClipped(double fl, double fr, double bl, double br) {
        setClippedMotorPower(frontLeft, fl);
        setClippedMotorPower(frontRight, fr);
        setClippedMotorPower(backLeft, bl);
        setClippedMotorPower(backRight, br);
    }

    public void driveTicks(int ticks, double power) {
        int targetPosition = frontRight.getCurrentPosition() + ticks;
        while ((targetPosition - frontRight.getCurrentPosition()) > 10) {
            double currentPower = power;
            driveMotors(power, power, power, power);
        }
        driveMotors(0, 0, 0, 0);
    }

    /*
     * sensor functions - hub
     */
    public LynxGetBulkInputDataResponse getBulkData(LynxModule hub) {
        LynxGetBulkInputDataCommand bulkInputDataCommand = new LynxGetBulkInputDataCommand(hub);
        try {
            return bulkInputDataCommand.sendReceive();
        } catch (InterruptedException | LynxNackException e) {
            e.printStackTrace();
        }
        return null;
    }

    /*
     * sensor functions - imu
     */
    public double getHeading() {
        return imu.getAngularOrientation(AxesReference.EXTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle; // - headingOffset; // navX.getYaw() - headingOffset; //
    }

    public void resetHeading() {
//        headingOffset = navX.getYaw();
    }

    public void lessBadTurn(double targetHeading) {
        lessBadTurn(targetHeading, 2);
    }

    public void lessBadTurn(double targetHeading, double timeout) {
        PIDController pid = new PIDController(new PIDCoefficients(0.032, 0.00005, 0.36), true, 1);

        ElapsedTime timer = new ElapsedTime();
        int correctFrames = 0;

        timer.reset();

        while (_opMode.opModeIsActive() && timer.seconds() < timeout) {
            /*
             * pid control loop
             */
            double currentHeading = getHeading();

            if (Math.abs(currentHeading - targetHeading) < 0.25) {
                currentHeading = targetHeading;
            }

            if (Math.abs(currentHeading - targetHeading) < 0.5) {
                correctFrames += 1;
                if (correctFrames > 20) {
                    break;
                }
            } else {
                correctFrames = 0;
            }

            double output = pid.step(currentHeading, targetHeading);

            driveMotors(-output, output, -output, output);

            _opMode.telemetry.addData("Target", targetHeading);
            _opMode.telemetry.addData("Current", currentHeading);
            _opMode.telemetry.addData("Output", output);
            _opMode.telemetry.addData("Correct frames", correctFrames);
            _opMode.telemetry.update();
        }

        driveMotors(0, 0, 0,0);
    }

    /*
     * sensor functions - tensorflow
     */
    public void activateTfod() {
        tfod.activate();
    }

    public void deactivateTfod() {
        tfod.shutdown();
    }

    public MineralPosition findGoldMineral() {
        List<Recognition> updatedRecognitions = tfod.getUpdatedRecognitions();

        if (updatedRecognitions == null || updatedRecognitions.size() != 3) {
            return MineralPosition.UNKNOWN;
        }

        int goldX, silver1X, silver2X;
        goldX = silver1X = silver2X = -1;

        for (Recognition r : updatedRecognitions) {
            if (r.getLabel().equals(Consts.TFOD_LABEL_GOLD)) {
                goldX = (int) r.getLeft();
            } else {
                if (silver1X == -1) {
                    silver1X = (int) r.getLeft();
                } else {
                    silver2X = (int) r.getLeft();
                }
            }
        }

        if (goldX == -1 || silver1X == -1 || silver2X == -1) {
            // shouldn't happen, but if it does, give up
            return MineralPosition.UNKNOWN;
        }

        if (goldX < silver1X && goldX < silver2X) {
            return MineralPosition.LEFT;
        } else if (goldX > silver1X && goldX > silver2X) {
            return MineralPosition.RIGHT;
        } else {
            return MineralPosition.CENTER;
        }
    }

    public MineralPosition findGoldMineralDifferent() {
        List<Recognition> updatedRecognitions = tfod.getUpdatedRecognitions();

        if (updatedRecognitions == null) {
            return MineralPosition.UNKNOWN;
        }

        for (Recognition r : updatedRecognitions) {
            if (r.getLabel().equals(Consts.TFOD_LABEL_GOLD)) {
                if (r.getLeft() > 200 && r.getLeft() < 400) {
                    return MineralPosition.CENTER;
                } else if (r.getLeft() <= 200) {
                    return MineralPosition.LEFT;
                } else if (r.getLeft() >= 400) {
                    return MineralPosition.RIGHT;
                }
            }

        }

        return MineralPosition.UNKNOWN;
    }

    // UTILITES
    public enum Direction {
        FORWARD, BACKWARD, LEFT, RIGHT;
    }

    public double rightDistance(Direction direction) {
        switch (direction) {
            case FORWARD:
                return rangeFrontRight.cmUltrasonic() * 10;
            case RIGHT:
                return rangeBackRight.cmUltrasonic() * 10;
            case LEFT:
                return rangeFrontLeft.cmUltrasonic() * 10;
            case BACKWARD:
                return rangeBackLeft.cmUltrasonic() * 10;
        }
        return 0;
    }
    public double leftDistance(Direction direction) {
        switch (direction) {
            case FORWARD:
                return rangeFrontLeft.cmUltrasonic() * 10;
            case RIGHT:
                return rangeFrontRight.cmUltrasonic() * 10;
            case LEFT:
                return rangeBackLeft.cmUltrasonic() * 10;
            case BACKWARD:
                return rangeBackRight.cmUltrasonic() * 10;
        }
        return 0;
    }
    public void setupSimpleServos(Direction direction) throws InterruptedException {
        double maxDif = 0;
        switch (direction) {
            case FORWARD:
                maxDif = Math.max(Math.abs(frontRightServo.getPosition()-SENSOR_SERVO_ZERO), Math.abs(frontLeftServo.getPosition()-SENSOR_SERVO_ZERO));
                frontRightServo.setPosition(SENSOR_SERVO_ZERO);
                frontLeftServo.setPosition(SENSOR_SERVO_ZERO);
            case RIGHT:
                maxDif = Math.max(Math.abs(frontRightServo.getPosition()-SENSOR_SERVO_FULL), Math.abs(backRightServo.getPosition()-SENSOR_SERVO_FULL));
                frontRightServo.setPosition(SENSOR_SERVO_FULL);
                backRightServo.setPosition(SENSOR_SERVO_FULL);
            case LEFT:
                maxDif = Math.max(Math.abs(frontLeftServo.getPosition()-SENSOR_SERVO_FULL), Math.abs(backLeftServo.getPosition()-SENSOR_SERVO_FULL));
                frontLeftServo.setPosition(SENSOR_SERVO_FULL);
                backLeftServo.setPosition(SENSOR_SERVO_FULL);
            case BACKWARD:
                maxDif = Math.max(Math.abs(backRightServo.getPosition()-SENSOR_SERVO_ZERO), Math.abs(frontLeftServo.getPosition()-SENSOR_SERVO_ZERO));
                backRightServo.setPosition(SENSOR_SERVO_ZERO);
                backLeftServo.setPosition(SENSOR_SERVO_ZERO);
        }
        double sleepTime = 210*(maxDif/Math.abs(SENSOR_SERVO_FULL - SENSOR_SERVO_ZERO))+50;
        Thread.sleep((long) sleepTime);
    }

    public void logSensors() {
        _opMode.telemetry.addData("range front right optical (cm)", rangeFrontRight.cmOptical());
        _opMode.telemetry.addData("range front right ultrasonic (cm)", rangeFrontRight.cmUltrasonic());

        _opMode.telemetry.addData("range back right optical (cm)", rangeBackRight.cmOptical());
        _opMode.telemetry.addData("range back right ultrasonic (cm)", rangeBackRight.cmUltrasonic());

        //_opMode.telemetry.addData("range front left optical (cm)", rangeFrontLeft.cmOptical());
        //_opMode.telemetry.addData("range front left ultrasonic (cm)", rangeFrontLeft.cmUltrasonic());

        //_opMode.telemetry.addData("range back left optical (cm)", rangeBackLeft.cmOptical());
        //_opMode.telemetry.addData("range back left ultrasonic (cm)", rangeBackLeft.cmUltrasonic());

        _opMode.telemetry.update();
    }
}
