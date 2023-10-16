//package org.firstinspires.ftc.teamcode.hardware;
//
//import com.qualcomm.robotcore.hardware.HardwareMap;
//
//public class viperslideFSMBACKUP implements Mechanism {
//
//    HardwareMap hwMap;
//    viperslideBACKUP slide = new viperslideBACKUP();
//    public enum States {
//        LOW,
//        MID,
//        HIGH,
//        BOTTOM,
//    }
//    public States state;
//
//    @Override
//    public void init(HardwareMap hwMap) {
//        this.hwMap = hwMap;
//        slide.init(hwMap);
//        state = States.BOTTOM;
//    }
//
//    public void run() {
//        switch(state) {
//            case LOW:
//                slide.setTargetPos(Lift.floor2);
//                break;
//            case MID:
//                slide.setTargetPos(Lift.floor3);
//                break;
//            case HIGH:
//                slide.setTargetPos(Lift.floor4);
//                break;
//            case BOTTOM:
//                slide.setTargetPos(Lift.floor1);
//                break;
//        }
//        if (!slide.targetReached())
//            slide.PIDUpdate();
//        else
//            slide.setPower(0);
//    }
//
//    public void high() {
//        state = state.HIGH;
//    }
//    public void mid() {
//        state = state.MID;
//    }
//    public void low() {
//        state = state.LOW;
//    }
//    public void bottom() {
//        state = state.BOTTOM;
//    }
//    public boolean targetReached() {
//        return slide.targetReached();
//    }
//}