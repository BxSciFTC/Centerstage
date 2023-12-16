package org.firstinspires.ftc.teamcode.opMode.testing.armTesting;

import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.teamcode.hardware.Mechanism;
import org.firstinspires.ftc.teamcode.hardware.NewArm;
import org.firstinspires.ftc.teamcode.hardware.RobotConstants;

import static java.lang.Math.*;


//formulaic mapping of motors and angles to x y coordinate
public class ArmMapper2 implements Mechanism {

    public NewArm2 arm;
    HardwareMap hwMap;

    //is q1 angle in the model
    public static double shoulderAngle;
    public static double elbowAngle;


    //is q2 angle in the model, i.e. acute angle between 2 lengths
    public static double elbowDownAngle;

    public static double x, y;

    public DigitalChannel redLED;
    public DigitalChannel greenLED;

    @Override
    public void init(HardwareMap hwMap) {
        this.hwMap = hwMap;
        arm = new NewArm2();
        arm.init(hwMap);

        x = 0;
        y = 20;

        redLED = hwMap.get(DigitalChannel.class, "red");
        greenLED = hwMap.get(DigitalChannel.class, "green");
        redLED.setMode(DigitalChannel.Mode.OUTPUT);
        greenLED.setMode(DigitalChannel.Mode.OUTPUT);
        greenLED.setState(true);
        redLED.setState(false);
    }
    /*
        Forward is positive x
        Up is positive y
    */

    public void moveTo(double x, double y) {
        double[] angles = calculateAngle(x, y);

//        if (strainedAngles(angles)) return;

        shoulderAngle = angles[0];
        elbowAngle = angles[1];
        elbowDownAngle = angles[2];

        this.x = x;
        this.y = y;

//        greenLED.setState(false);
//        redLED.setState(true);
    }

    public void shift(double x1, double y1) {
        double newX = isInRange(x + x1, y) ? x + x1 : x;
        double newY = isInRange(newX, y + y1) ? y + y1 : y;

        this.x = newX;
        this.y = newY;

//        moveTo(newX, newY);
        arm.shoulder.setPower(1);

        redLED.setState(false);
        greenLED.setState(true);
    }

    //tells if coordinate is outside the range of motion of the two arms(a circle shape)
    public boolean isInRange(double x, double y) {
        double totalLen = 20;
        return !(abs(x) > abs(cos(atan2(y, x))) * totalLen || abs(y) > abs(sin(atan2(y, x)))* totalLen);
    }

//    //we keep q1 bounded 0-90 and 180-q2 bounded 0-180
//    public boolean strainedAngles(double[] angles) {
//        return (angles[0] < 0 || angles[0] > 90 || angles[1] > 180 || angles[1] < 10);
//    }

    public void PIDUpdate() {
        arm.shoulderGoToAngle(shoulderAngle);
        arm.elbowGoToAngle(elbowAngle);
        arm.PIDUpdate();
    }

    //elbow angle will be DOWN angle
    //shoulder angle will be angle of elevation
    public double[] calculateAngle(double x, double y) {
        double a = 10;
        double b = 10;
        double armAngle = -1 * acos(
                (pow(x, 2) + pow(y, 2) - pow(a, 2) - pow(b, 2))/
                        (2 * a * b)
        );
        double shoulderAngle = atan(y / x) +atan (
                (b * sin(armAngle)) /
                        (a + b * cos(armAngle))

        );
        //q1, angle between arms, and q2 in the model
        return new double[] {Math.toDegrees(shoulderAngle), 180-Math.toDegrees(armAngle),Math.toDegrees(armAngle)};
    }
}
