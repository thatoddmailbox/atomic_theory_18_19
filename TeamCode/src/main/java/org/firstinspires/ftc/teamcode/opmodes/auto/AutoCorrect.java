package org.firstinspires.ftc.teamcode.opmodes.auto;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;

import org.firstinspires.ftc.teamcode.robot.Robot;
import org.firstinspires.ftc.teamcode.utils.PowerSetting;
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
     * Get difference between current rotation and original rotation.
     * @return Angle in degrees. + = left, - = right?
     */
    private double getDeltaAngle(Robot robot) {
        double angle = robot.getHeading();

        if (firstAngle > 360) {
            firstAngle = angle;
        }

        double deltaAngle = angle - firstAngle;

        return deltaAngle;
    }

    /**
     * See if we are moving in a straight line and if not return a power correction value.
     * @return Power adjustment, + is adjust left - is adjust right?
     */
    private double checkDirection(Robot robot) {
        double angle = getDeltaAngle(robot);

        double percentageError = (angle/180)*100;
        double sign = 1;
        if (percentageError < 0) {
            sign = -1;
            percentageError = -percentageError;
        }

        // Add one to percentage error to translate log graph 1 unit left
        //        double correction = sign * (Math.log((percentageError+2)/2)/2)/100;
        double correction = sign * (Math.log((percentageError+2)/2)/2)/100;

        return correction;
    }
}