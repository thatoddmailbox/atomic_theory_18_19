package org.firstinspires.ftc.teamcode.robot;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;
import org.firstinspires.ftc.teamcode.Consts;
import org.firstinspires.ftc.teamcode.utils.MineralPosition;

import java.util.List;

public class Robot {
    public DcMotor frontLeft;
    public DcMotor frontRight;
    public DcMotor backLeft;
    public DcMotor backRight;

    public DcMotor george;
    public DcMotor lenny;
    public DcMotor latch;

    public BNO055IMU imu;

    public WebcamName leftWebcam;
    public WebcamName rightWebcam;

    public DcMotor.ZeroPowerBehavior driveMotorZeroPowerBehavior;

    public VuforiaLocalizer vuforia;
    public TFObjectDetector tfod;

    public Robot(LinearOpMode opMode) throws InterruptedException {
        /*
         * motor initialization
        */
        frontLeft = opMode.hardwareMap.dcMotor.get("front_left");
        frontRight = opMode.hardwareMap.dcMotor.get("front_right");
        backLeft = opMode.hardwareMap.dcMotor.get("back_left");
        backRight = opMode.hardwareMap.dcMotor.get("back_right");

        george = opMode.hardwareMap.dcMotor.get("george");
        lenny = opMode.hardwareMap.dcMotor.get("lenny");
        latch = opMode.hardwareMap.dcMotor.get("latch");

        /*
         * motor setup
         */
        frontLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        frontRight.setDirection(DcMotorSimple.Direction.FORWARD);
        backLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        backRight.setDirection(DcMotorSimple.Direction.FORWARD);

        george.setDirection(DcMotorSimple.Direction.FORWARD);
        lenny.setDirection(DcMotorSimple.Direction.FORWARD);
        latch.setDirection(DcMotorSimple.Direction.FORWARD);

        setDriveMotorZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        george.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        lenny.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        latch.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

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

        /*
         * initialize sensors
         */
        VuforiaLocalizer.Parameters vuforiaParameters = new VuforiaLocalizer.Parameters();
        vuforiaParameters.vuforiaLicenseKey = Consts.VUFORIA_KEY;
        vuforiaParameters.cameraName = leftWebcam;
        vuforia = ClassFactory.getInstance().createVuforia(vuforiaParameters);

        int tfodMonitorViewId = opMode.hardwareMap.appContext.getResources().getIdentifier("tfodMonitorViewId", "id", opMode.hardwareMap.appContext.getPackageName());
        TFObjectDetector.Parameters tfodParameters = new TFObjectDetector.Parameters(tfodMonitorViewId);
        tfod = ClassFactory.getInstance().createTFObjectDetector(tfodParameters, vuforia);
        tfod.loadModelFromAsset(Consts.TFOD_MODEL_FILE, Consts.TFOD_LABEL_GOLD, Consts.TFOD_LABEL_SILVER);

//        while (!imu.isGyroCalibrated()) {
//            opMode.telemetry.addLine("Calibrating gyro...");
//            opMode.telemetry.update();
//
//            Thread.sleep(50);
//            opMode.idle();
//        }
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
}
