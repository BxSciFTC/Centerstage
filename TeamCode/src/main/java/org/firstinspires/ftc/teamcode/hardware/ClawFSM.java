package org.firstinspires.ftc.teamcode.hardware;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.HardwareMap;

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


    public HingeState hinge;
    public enum HingeState {
        ZEROANGLE,
        THIRTYANGLE,
    }




    public static double switchHinge = 10;

    @Override
    public void init(HardwareMap hwMap) {
        this.hwMap = hwMap;
        clawMechanism.init(hwMap);
        leftClawState = LeftClawState.OPEN;
        rightClawState=RightClawState.OPEN;
        hinge = HingeState.ZEROANGLE;
    }

    public void leftOpen(){
        leftClawState = LeftClawState.OPEN;
    }
    public void leftClose(){
        leftClawState = LeftClawState.CLOSE;
    }

    public void rightOpen(){
        rightClawState = rightClawState.OPEN;
    }
    public void rightClose(){
        rightClawState = rightClawState.CLOSE;
    }

    public void setZeroAngle(){
        hinge = HingeState.ZEROANGLE; //BRYAN THIS JUST SETS THE STATE, IT DOES NOT DO ANYTHING to change the servo POSTION. IF YOU CHANGE MY FUNNY CODE I WILL COMMIT COOKIE..
    }
    public void setThirtyAngle(){
        hinge = HingeState.THIRTYANGLE;
    }

    public void update() {
        if(ArmMapper.yPos < switchHinge){
            setZeroAngle();
        }
        else{
            setThirtyAngle();
        }

        switch (hinge) {
            case ZEROANGLE:
                clawMechanism.zeroAngleConstant();
                break;
            case THIRTYANGLE:
                clawMechanism.thirtyAngleConstant();
                break;
        }
        switch (leftClawState) {
            case OPEN:
                clawMechanism.leftOpen();
            case CLOSE:
                clawMechanism.leftClose();
        }
        switch (rightClawState) {
            case OPEN:
                clawMechanism.rightOpen();
            case CLOSE:
                clawMechanism.rightClose();
        }
    }
}