package org.firstinspires.ftc.teamcode.hardware;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

@Config
public class Claw implements Mechanism {
    HardwareMap hwMap;
    Servo leftClawController;
    Servo rightClawController;
    Servo hingeController;

    public static double elbowAngle;
    public static double shoulderAngle;

    public static double open1 = 0.1;
    public static double close1 = 0.5;

    public static double open2 = 0.8;
    public static double close2 = 0.35;

    public static double restPos = 0;

    boolean leftIsOpen = false;
    boolean rightIsOpen = false;

    @Override
    public void init(HardwareMap hwMap) {
        this.hwMap = hwMap;
        leftClawController = hwMap.get(Servo.class, "leftClaw");
        rightClawController =
                hwMap.get(Servo.class, "rightClaw");
        hingeController = hwMap.get(Servo.class, "hinge");
        elbowAngle = ArmMapper.elbowAngle;
        shoulderAngle = ArmMapper.shoulderAngle;
    }
    public void leftOpen() {
        leftClawController.setPosition(open1);
        leftIsOpen = true;
    }

    public void leftClose() {
        leftClawController.setPosition(close1);
        leftIsOpen = false;
    }

    public void rightOpen() {
        rightClawController.setPosition(open2);
        rightIsOpen = true;
    }

    public void rightClose() {
        rightClawController.setPosition(close2);
        rightIsOpen = false;
    }

    public void thirtyAngleVariable(){
        double elbowDownAngle = 180 - shoulderAngle - elbowAngle;
        double a = 90 - elbowDownAngle;
        double servoAngle = a + 30;
        hingeController.setPosition(Func.map(servoAngle, -90, 180, -1, 1));
    }

    public void zeroAngleVariable(){
        double elbowDownAngle = 180 - shoulderAngle - elbowAngle;
        double a = 90 - elbowDownAngle;
        double servoAngle = a + 90;
        hingeController.setPosition(Func.map(servoAngle, -90, 180, -1, 1));
    }

    public void restAngleConstant() {
        hingeController.setPosition(restPos);
    }

    public void update() {
        elbowAngle = ArmMapper.elbowAngle;
        shoulderAngle = ArmMapper.shoulderAngle;
    }
}


