package org.firstinspires.ftc.teamcode.hardware;

import com.qualcomm.robotcore.hardware.HardwareMap;

import static java.lang.Math.*;


//formulaic mapping of motors and angles to x y coordinate
public class ArmMapper implements Mechanism {

    Arm arm = new Arm();
    HardwareMap hwMap;

    public static double shoulderAngle;
    public static double elbowAngle;

    //TODO: calculate
    public static double elbowDownAngle;
    public static double x, y;

    @Override
    public void init(HardwareMap hwMap) {
        this.hwMap = hwMap;
        arm.init(hwMap);
        shoulderAngle = arm.shoulderDegrees();
        elbowAngle = arm.elbowDegrees();
        elbowDownAngle = (90 - elbowAngle - (90 - shoulderAngle));
        //x, y
        double vector1[] = {RobotConstants.shoulderLen*cos(shoulderAngle), RobotConstants.shoulderLen*sin(shoulderAngle)};
        double vector2[] = {RobotConstants.elbowLen*cos(-elbowDownAngle), RobotConstants.elbowLen*sin(-elbowDownAngle)};
        x = vector1[0] + vector2[0];
        y = vector1[0] + vector2[0];
    }
    /*
        Forward is positive x
        Up is positive y
    */

    public void moveTo(double x, double y) {
        double[] angles = calculateAngle(x, y);

        if (angles[0] < 0 || angles[0] > 90 || calculateElbowAngle(angles[0], angles[1]) < 10)
            return;

        shoulderAngle = angles[0];
        elbowAngle = calculateElbowAngle(angles[0], angles[1]);
        elbowDownAngle = angles[1];
        arm.shoulderGoToAngle(shoulderAngle);
        arm.elbowGoToAngle(elbowAngle);
        this.x = x;
        this.y = y;
    }

    public void shift(double x, double y) {
        double newX = this.x + x;
        double newY = this.y + y;
        double[] angles = calculateAngle(newX,newY);

        if (angles[0] < 0 || angles[0] > 90 || calculateElbowAngle(angles[0], angles[1]) < 10)
            return;

        shoulderAngle = angles[0];
        elbowAngle = calculateElbowAngle(angles[0], angles[1]);
        elbowDownAngle = angles[1];
        arm.shoulderGoToAngle(shoulderAngle);
        arm.elbowGoToAngle(elbowAngle);
        this.x =  newX;
        this.y = newY;

    }

    public double calculateElbowAngle(double shoulder, double elbowDown) {
        return 180 - shoulder - elbowDown;
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
        return new double[] {shoulderAngle, armAngle};
    }

}
