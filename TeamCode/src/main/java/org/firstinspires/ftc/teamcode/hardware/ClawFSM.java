package org.firstinspires.ftc.teamcode.hardware;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class ClawFSM implements Mechanism {
    HardwareMap hwMap;
    Claw ClawMechanism = new Claw();

    public enum ClawState{
        OPEN,
        CLOSE,
    }
    public enum HingeState{
        ZEROANGLE,
        THIRTYANGLE,
    }
    public ClawState clawS;
    public HingeState hinge;
    @Override
    public void init(HardwareMap hwMap) {
        this.hwMap = hwMap;
        ClawMechanism.init(hwMap);
        clawS = ClawState.OPEN;
        hinge = HingeState.ZEROANGLE;
        //ArmMapper.elbowDownAngle;
    }
    public void open(){
        clawS = ClawState.OPEN;
    }
    public void close(){
        clawS = ClawState.CLOSE;
    }




}