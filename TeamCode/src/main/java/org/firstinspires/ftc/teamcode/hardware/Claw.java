package org.firstinspires.ftc.teamcode.hardware;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

@Config
public class Claw implements Mechanism {
    HardwareMap hwMap;
    Servo clawController;
    Servo hingeController;

    public static double elbowAngle;
    public static double shoulderAngle;

    public static double open = 0.5;
    public static double close = -0.5;

    boolean isOpen;
    @Override
    public void init(HardwareMap hwMap) {
        this.hwMap = hwMap;
        clawController = hwMap.get(Servo.class, "claw");
        hingeController = hwMap.get(Servo.class, "hinge");
        elbowAngle = ArmMapper.elbowAngle;
        shoulderAngle = ArmMapper.shoulderAngle;
    }
    public void open() {
        clawController.setPosition(open);
        isOpen = true;
    }

    public void close() {
        clawController.setPosition(close);
        isOpen = false;
    }

    public void thirtyAngleConstant(){
        double elbowDownAngle = 180 - shoulderAngle - elbowAngle;
        double a = 90 - elbowDownAngle;
        double servoAngle = a + 30;
        hingeController.setPosition(Func.map(servoAngle, -90, 180, -1, 1));
    }

    public void zeroAngleConstant(){
        double elbowDownAngle = 180 - shoulderAngle - elbowAngle;
        double a = 90 - elbowDownAngle;
        double servoAngle = a + 90;
        hingeController.setPosition(Func.map(servoAngle, -90, 180, -1, 1));
    }
}


