package org.firstinspires.ftc.teamcode.hardware;

import com.qualcomm.robotcore.hardware.HardwareMap;

//all hardware controls for both teleOp and Auton
public class Robot implements Mechanism {
    HardwareMap hwMap;
    ArmClawFSM arm;
    Lift lift;
    Plane plane;
    @Override
    public void init(HardwareMap hwMap) {
        this.hwMap = hwMap;
        arm = new ArmClawFSM();
        arm.init(hwMap);
        lift = new Lift();
        lift.init(hwMap);
        plane = new Plane();
        plane.init(hwMap);
    }

    public void armShift(double x, double y) {
        arm.shift(x, y);
    }

}
