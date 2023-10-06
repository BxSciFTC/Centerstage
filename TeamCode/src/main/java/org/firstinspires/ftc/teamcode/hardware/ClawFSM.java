package org.firstinspires.ftc.teamcode.hardware;

import com.qualcomm.robotcore.hardware.HardwareMap;

public class ClawFSM implements Mechanism {
    HardwareMap hwMap;
    Claw clawMechanism = new Claw();

    public enum ClawState {
        OPEN,
        CLOSE,
    }

    public enum HingeState {
        ZEROANGLE,
        THIRTYANGLE,
    }

    public ClawState clawS;
    public HingeState hinge;

    @Override
    public void init(HardwareMap hwMap) {
        this.hwMap = hwMap;
        clawMechanism.init(hwMap);
        clawS = ClawState.OPEN;
        hinge = HingeState.ZEROANGLE;
    }

    public void open(){
        clawS = ClawState.OPEN;
    }
    public void close(){
        clawS = ClawState.CLOSE;
    }

    public void setZeroAngle(){
        hinge = HingeState.ZEROANGLE; //BRYAN THIS JUST SETS THE STATE, IT DOES NOT DO ANYTHING to change the servo POSTION. IF YOU CHANGE MY FUNNY CODE I WILL COMMIT COOKIE..
    }
    public void setThirtyAngle(){
        hinge = HingeState.THIRTYANGLE;
    }



    public void update() {
        switch (hinge) {
            case ZEROANGLE:
                clawMechanism.zeroAngleConstant();
                break;
            case THIRTYANGLE:
                clawMechanism.thirtyAngleConstant();
                break;
        }
        switch (clawS) {
            case OPEN:
                clawMechanism.open();
            case CLOSE:
                clawMechanism.close();
        }
    }



}