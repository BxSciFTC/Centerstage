package org.firstinspires.ftc.teamcode.hardware;

import com.qualcomm.robotcore.hardware.HardwareMap;

//all hardware controls for both teleOp and Auton
public class Robot implements Mechanism {
    HardwareMap hwMap;
    @Override
    public void init(HardwareMap hwMap) {
        this.hwMap = hwMap;
    }
}
