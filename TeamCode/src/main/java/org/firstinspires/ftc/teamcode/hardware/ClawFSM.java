package org.firstinspires.ftc.teamcode.hardware;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.teamcode.opMode.teleOp.TeleOpMain;

@Config
public class ClawFSM implements Mechanism {
    HardwareMap hwMap;
    Claw clawMechanism = new Claw();

    public LeftClawState leftClawState;
    public enum LeftClawState {
        OPEN,
        CLOSE,
    }

    public RightClawState rightClawState;
    public enum RightClawState {
        OPEN,
        CLOSE,
    }

    public HingeState hingeState;
    public enum HingeState {
        REST,
        PREAUTONREST,
        PICKUP,
        SCORE1,
        SCORE2,
    }


    @Override
    public void init(HardwareMap hwMap) {
        this.hwMap = hwMap;
        clawMechanism.init(hwMap);
        leftClawState = LeftClawState.CLOSE;
        rightClawState= RightClawState.CLOSE;
        hingeState = HingeState.REST;
    }

    public void leftOpen(){
        leftClawState = leftClawState.OPEN;
    }
    public void leftClose(){
        leftClawState = leftClawState.CLOSE;
    }

    public void rightOpen(){
        rightClawState = rightClawState.OPEN;
    }
    public void rightClose(){
        rightClawState = rightClawState.CLOSE;
    }


    public void setPreAutonRest() {
        hingeState = HingeState.PREAUTONREST;
    }
    public void setRest() {
        hingeState = HingeState.REST;
    }
    public void setPickup() {
        hingeState = HingeState.PICKUP;
    }
    public void setScore1() {
        hingeState = HingeState.SCORE1;
    }
    public void setScore2() {
        hingeState = HingeState.SCORE2;
    }

    public void update() {
        TeleOpMain.tele.addData(String.valueOf(hingeState), String.valueOf(hingeState));
        switch (hingeState) {
            case PREAUTONREST:
                TeleOpMain.tele.addData("1", "1");
                clawMechanism.preAutonomousRest();
                break;
            case REST:
                TeleOpMain.tele.addData("2", "1");
                clawMechanism.restAngleConstant();
                break;
            case PICKUP:
                TeleOpMain.tele.addData("3", "1");
                clawMechanism.pickup();
                break;
            case SCORE1:
                TeleOpMain.tele.addData("4", "1");
                clawMechanism.score1();
                break;
            case SCORE2:
                TeleOpMain.tele.addData("5", "1");
                clawMechanism.score2();
                break;
        }
        switch (leftClawState) {
            case OPEN:
                clawMechanism.leftOpen();
                break;
            case CLOSE:
                clawMechanism.leftClose();
                break;
        }
        switch (rightClawState) {
            case OPEN:
                clawMechanism.rightOpen();
                break;
            case CLOSE:
                clawMechanism.rightClose();
                break;
        }
        TeleOpMain.tele.update();
    }

}

//package org.firstinspires.ftc.teamcode.hardware;
//
//import com.acmerobotics.dashboard.config.Config;
//import com.qualcomm.robotcore.hardware.HardwareMap;
//
//@Config
//public class ClawFSM implements Mechanism {
//    HardwareMap hwMap;
//    Claw clawMechanism = new Claw();
//
//    public LeftClawState leftClawState;
//    public enum LeftClawState {
//        OPEN,
//        CLOSE,
//    }
//
//    public RightClawState rightClawState;
//    public enum RightClawState {
//        OPEN,
//        CLOSE,
//    }
//
//    public HingeState hingeState;
//    public enum HingeState {
//        ZEROANGLE,
//        THIRTYANGLE,
//        REST,
//    }
//
//    public static double switchHinge = 6;
//
//    @Override
//    public void init(HardwareMap hwMap) {
//        this.hwMap = hwMap;
//        clawMechanism.init(hwMap);
//        leftClawState = LeftClawState.CLOSE;
//        rightClawState= RightClawState.CLOSE;
//        hingeState = HingeState.ZEROANGLE;
//    }
//
//    public void leftOpen(){
//        leftClawState = leftClawState.OPEN;
//    }
//    public void leftClose(){
//        leftClawState = leftClawState.CLOSE;
//    }
//
//    public void rightOpen(){
//        rightClawState = rightClawState.OPEN;
//    }
//    public void rightClose(){
//        rightClawState = rightClawState.CLOSE;
//    }
//
//    public void setZeroAngle(){
//        hingeState = HingeState.ZEROANGLE;
//    }
//    public void setThirtyAngle(){
//        hingeState = HingeState.THIRTYANGLE;
//    }
//    public void setRestAngle() {
//        hingeState = HingeState.REST;
//    }
//
//    public void update() {
//        clawMechanism.update();
//
//        if (hingeState == HingeState.REST) {
//
//        } else {
//            if (ArmMapper.yPos < switchHinge) {
//                setZeroAngle();
//            } else {
//                setThirtyAngle();
//            }
//        }
//
//        switch (hingeState) {
//            case ZEROANGLE:
//                clawMechanism.zeroAngleVariable();
//                break;
//            case THIRTYANGLE:
//                clawMechanism.thirtyAngleVariable();
//                break;
//            case REST:
//                clawMechanism.restAngleConstant();
//                break;
//        }
//        switch (leftClawState) {
//            case OPEN:
//                clawMechanism.leftOpen();
//                break;
//            case CLOSE:
//                clawMechanism.leftClose();
//                break;
//        }
//        switch (rightClawState) {
//            case OPEN:
//                clawMechanism.rightOpen();
//                break;
//            case CLOSE:
//                clawMechanism.rightClose();
//                break;
//        }
//    }
//}