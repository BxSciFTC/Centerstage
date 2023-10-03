package org.firstinspires.ftc.teamcode.hardware;

import com.qualcomm.robotcore.hardware.HardwareMap;

//all hardware controls for both teleOp and Auton
public class Robot implements Mechanism {
    HardwareMap hwMap;
    ArmMapper arm = new ArmMapper();
    @Override
    public void init(HardwareMap hwMap) {
        this.hwMap = hwMap;
        arm.init(hwMap);
    }

    public void armShift(double x, double y) {
        arm.shift(x, y);
    }

}
