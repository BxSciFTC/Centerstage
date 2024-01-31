package org.firstinspires.ftc.teamcode.hardware;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import org.firstinspires.ftc.teamcode.opMode.teleOp.TeleOpMain;

@Config
public class Claw implements Mechanism {
    HardwareMap hwMap;
    Servo leftClawController;
    Servo rightClawController;
    Servo hingeController;

    public double open2 = 0.1;
    public double close2 = 0.5;

    public double open1 = 0.8;
    public double close1 = 0.35;

    public static double preAutonRest = 1;
    public static double restPos = 0;
    public static double pickup = 0;
    public static double score1 = 1;
    public static double score2 = 0.5;

    boolean leftIsOpen = false;
    boolean rightIsOpen = false;

    @Override
    public void init(HardwareMap hwMap) {
        this.hwMap = hwMap;
        leftClawController = hwMap.get(Servo.class, "leftClaw");
        rightClawController =
                hwMap.get(Servo.class, "rightClaw");
        hingeController = hwMap.get(Servo.class, "hinge");
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

//    public void thirtyAngleVariable(){
//        double elbowDownAngle = 180 - shoulderAngle - elbowAngle;
//        double a = 90 - elbowDownAngle;
//        double servoAngle = a + 30;
//        hingeController.setPosition(Func.map(servoAngle, -90, 180, -1, 1));
//    }
//
//    public void zeroAngleVariable(){
//        double elbowDownAngle = 180 - shoulderAngle - elbowAngle;
//        double a = 90 - elbowDownAngle;
//        double servoAngle = a + 90;
//        hingeController.setPosition(Func.map(servoAngle, -90, 180, -1, 1));
//    }

    public void preAutonomousRest() {
        TeleOpMain.tele.addData("111", "1");

//        hingeController.setPosition(preAutonRest);
    }
    public void restAngleConstant() {
        TeleOpMain.tele.addData("222", "1");
//        hingeController.setPosition(restPos);
    }
    public void pickup() {
        TeleOpMain.tele.addData("333", "1");
//        hingeController.setPosition(pickup);
    }
    public void score1() {
        TeleOpMain.tele.addData("444", "1");
//        hingeController.setPosition(score1);
    }
    public void score2() {
        TeleOpMain.tele.addData("555", "1");
//        hingeController.setPosition(score2);
    }

    public void update() {
//        elbowAngle = ArmMapper.elbowAngle;
//        shoulderAngle = ArmMapper.shoulderAngle;
    }
}


