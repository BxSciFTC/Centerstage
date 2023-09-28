package org.firstinspires.ftc.teamcode.hardware;

import com.qualcomm.robotcore.hardware.HardwareMap;

import static java.lang.Math.*;


//formulaic mapping of motors and angles to x y coordinate
public class ArmMapper implements Mechanism {

    Arm arm = new Arm();
    HardwareMap hwMap;

    @Override
    public void init(HardwareMap hwMap) {
        this.hwMap = hwMap;
        arm.init(hwMap);
    }
    /*
        Forward is positive x
        Up is positive y
    */

    public void moveTo(double x, double y) {
        double[] angles = calculateAngle(x, y);
        arm.elbowGoToAngle(angles[0]);
        arm.elbowGoToAngle(angles[1]);
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
