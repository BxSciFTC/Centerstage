package org.firstinspires.ftc.teamcode.hardware;

import com.qualcomm.robotcore.hardware.HardwareMap;

import static java.lang.Math.*;


//formulaic mapping of motors and angles to x y coordinate
public class ArmMapper implements Mechanism {

    Arm arm = new Arm();
    HardwareMap hwMap;

    //is q1 angle in the model
    public static double shoulderAngle;
    public static double elbowAngle;


    //is q2 angle in the model
    public static double elbowDownAngle;
    public static double x, y;

    @Override
    public void init(HardwareMap hwMap) {
        this.hwMap = hwMap;
        arm.init(hwMap);
        shoulderAngle = arm.shoulderDegrees();
        elbowAngle = arm.elbowDegrees();
        elbowDownAngle = 180 - elbowAngle;

        //x, y
        double shoulderAngleDegrees = Math.toRadians(shoulderAngle);
        double vector1[] = {RobotConstants.shoulderLen*cos(shoulderAngleDegrees), RobotConstants.shoulderLen*sin(shoulderAngleDegrees)};
        double trueElbowAngle = elbowAngle - (90 - shoulderAngle) - 90;
        double trueElbowAngleDegrees = elbowAngle - (90 - shoulderAngle) - 90;
        double vector2[] = {RobotConstants.elbowLen*cos(trueElbowAngleDegrees), RobotConstants.elbowLen*sin(trueElbowAngleDegrees)};
        x = vector1[0] + vector2[0];
        y = vector1[1] + vector2[1];
    }
    /*
        Forward is positive x
        Up is positive y
    */

    public void moveTo(double x, double y) {
        double[] angles = calculateAngle(x, y);

        if (strainedAngles(angles)) return;

        shoulderAngle = angles[0];
        elbowAngle = angles[1];
        elbowDownAngle = angles[2];
        arm.shoulderGoToAngle(shoulderAngle);
        arm.elbowGoToAngle(elbowAngle);
        this.x = x;
        this.y = y;
    }

    public void shift(double x, double y) {
        double newX = this.x + x;
        double newY = this.y + y;
        double[] angles = calculateAngle(newX,newY);

        if (strainedAngles(angles)) return;

        shoulderAngle = angles[0];
        elbowAngle = angles[1];
        elbowDownAngle = angles[2];
        arm.shoulderGoToAngle(shoulderAngle);
        arm.elbowGoToAngle(elbowAngle);

        this.x =  newX;
        this.y = newY;
    }

    //we keep q1 bounded 0-90 and 180-q2 bounded 0-180
    public boolean strainedAngles(double[] angles) {
        return (angles[0] < 0 || angles[0] > 90 || angles[1] > 180 || angles[1] < 10);
    }

    public void PIDUpdate() {
        arm.PIDUpdate();
    }

    //elbow angle will be DOWN angle
    //shoulder angle will be angle of elevation
    public double[] calculateAngle(double x, double y) {
        double a = RobotConstants.shoulderLen;
        double b = RobotConstants.elbowLen;
        double armAngle = -1 * acos(
                (pow(x, 2) + pow(y, 2) - pow(a, 2) - pow(b, 2))/
                        (2 * a * b)
        );
        double shoulderAngle = atan(y / x) + atan(
                (b * sin(armAngle)) /
                        (a + b * cos(armAngle))

        );
        //q1, angle between arms, and q2 in the model
        return new double[] {shoulderAngle, 180-armAngle, armAngle};
    }

}
