package org.firstinspires.ftc.teamcode.hardware;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.HardwareMap;
@Config
public class ScuffedAngle implements Mechanism{
    public NewArm arm;
    HardwareMap hwMap;

    //is q1 angle in the model
    public static double shoulderAngleFloor;
    public static double elbowAngleFloor;
    public static double shoulderAngleDrop;
    public static double elbowAngleDrop;
    public static double shoulderAngleLow;
    public static double elbowAngleLow;
    public static double shoulderAngleHigh;
    public static double elbowAngleHigh;

    //is q2 angle in the model, i.e. acute angle between 2 lengths
    public double elbowDownAngle;

    public static double xPos, yPos;


    @Override
    public void init(HardwareMap hwMap) {
        this.hwMap = hwMap;
        arm = new NewArm();
        arm.init(hwMap);

    }
    public void floor(){
        arm.shoulderGoToAngle(shoulderAngleFloor);
        arm.elbowGoToAngle(elbowAngleFloor);
        arm.PIDUpdate();
    }
    public void drop(){

        arm.shoulderGoToAngle(shoulderAngleDrop);
        arm.elbowGoToAngle(elbowAngleDrop);
        arm.PIDUpdate();
    }
    public void low(){

        arm.shoulderGoToAngle(shoulderAngleLow);
        arm.elbowGoToAngle(elbowAngleLow);
        arm.PIDUpdate();
    }
    public void high(){

        arm.shoulderGoToAngle(shoulderAngleHigh);
        arm.elbowGoToAngle(elbowAngleHigh);
        arm.PIDUpdate();
    }
}
