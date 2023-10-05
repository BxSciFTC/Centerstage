package org.firstinspires.ftc.teamcode.hardware;

import com.qualcomm.robotcore.hardware.HardwareMap;

public class LiftFSM implements Mechanism {

    HardwareMap hwMap;
    Lift lift = new Lift();
    public enum States {
        LOW,
        MID,
        HIGH,
        BOTTOM,
    }
    public States state;

    @Override
    public void init(HardwareMap hwMap) {
        this.hwMap = hwMap;
        lift.init(hwMap);
        state = States.BOTTOM;
    }

    public void run() {
        switch(state) {
            case LOW:
                lift.setTargetPos(Lift.floor2);
                break;
            case MID:
                lift.setTargetPos(Lift.floor3);
                break;
            case HIGH:
                lift.setTargetPos(Lift.floor4);
                break;
            case BOTTOM:
                lift.setTargetPos(Lift.floor1);
                break;
        }
        if (!lift.targetReached())
            lift.PIDUpdate();
        else
            lift.setPower(0);
    }

    public void high() {
        state = state.HIGH;
    }
    public void mid() {
        state = state.MID;
    }
    public void low() {
        state = state.LOW;
    }
    public void bottom() {
        state = state.BOTTOM;
    }
    public boolean targetReached() {
        return lift.targetReached();
    }
}