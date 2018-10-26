package org.firstinspires.ftc.teamcode.opmodes.auto;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.hardware.DigitalChannel;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

import org.firstinspires.ftc.teamcode.robot.Robot;
import org.firstinspires.ftc.teamcode.utils.Alliance;
import org.firstinspires.ftc.teamcode.utils.PowerSetting;
import org.firstinspires.ftc.teamcode.utils.StartingPosition;
import java.lang.Math;

public class AutoCorrect {

    double firstAngle = 361;
    double correction;


    public void setCalibratedPower(PowerSetting targetPower, Robot robot) {
        correction = checkDirection(robot);

        targetPower.fl = targetPower.fl + correction;
        targetPower.fr = targetPower.fr - correction;
        targetPower.bl = targetPower.bl + correction;
        targetPower.br = targetPower.br - correction;

        robot.driveMotors(targetPower.fl, targetPower.fr, targetPower.bl, targetPower.br);
    }


    /**
     * Get current cumulative angle rotation from last reset.
     * @return Angle in degrees. + = left, - = right.
     */
    private double getDeltaAngle(Robot robot) {
        // We experimentally determined the Z axis is the axis we want to use for heading angle.
        // We have to process the angle because the imu works in euler angles so the Z axis is
        // returned as 0 to +180 or 0 to -180 rolling back to -179 or +179 when rotation passes
        // 180 degrees. We detect this transition and track the total cumulative angle of rotation.

        double angle = robot.imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle;

        if (firstAngle > 360) {
            firstAngle = angle;
        }

        double deltaAngle = angle - firstAngle;

//        if (deltaAngle < -180)
//            deltaAngle += 360;
//        else if (deltaAngle > 180)
//            deltaAngle -= 360;
//
//        globalAngle += deltaAngle;

        return deltaAngle;
    }

    /**
     * See if we are moving in a straight line and if not return a power correction value.
     * @return Power adjustment, + is adjust left - is adjust right.
     */
    private double checkDirection(Robot robot) {
        double correction, angle;

        angle = getDeltaAngle(robot);
        double percentageError = (angle/180)*100;
        double sign = 1;
        if (percentageError < 0) {
            sign = -1;
            percentageError = -percentageError;
        }
        correction = sign * Math.pow(percentageError, 0.6)/100;

        return correction;
    }
}