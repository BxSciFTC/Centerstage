package org.firstinspires.ftc.teamcode.hardware;

import com.qualcomm.robotcore.hardware.HardwareMap;

//all hardware controls for both teleOp and Auton
public class Robot implements Mechanism {
    HardwareMap hwMap;
    public ArmClawFSM arm;
    public Lift lift;
    public Plane plane;
    public MecanumDrive drive;

    @Override
    public void init(HardwareMap hwMap) {
        this.hwMap = hwMap;
        arm = new ArmClawFSM();
        arm.init(hwMap);
        lift = new Lift();
        lift.init(hwMap);
        plane = new Plane();
        plane.init(hwMap);
        drive = new MecanumDrive();
        drive.init(hwMap);
    }

    public void armShift(double x, double y) {
        arm.shift(x, y);
    }
}
