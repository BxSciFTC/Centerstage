package org.firstinspires.ftc.teamcode.hardware;

import com.qualcomm.robotcore.hardware.HardwareMap;

public class Lift implements Mechanism {
    HardwareMap hwMap;
    @Override
    public void init(HardwareMap hwMap) {
        this.hwMap = hwMap;
    }
}
