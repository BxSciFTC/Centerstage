package org.firstinspires.ftc.teamcode.hardware;

import com.qualcomm.robotcore.hardware.HardwareMap;

public class ArmClawFSM implements Mechanism {
    HardwareMap hwMap;
    NewArm arm;
    ClawFSM claw;

//    ScuffedAngle desperation;
    @Override
    public void init(HardwareMap hwMap) {
        this.hwMap = hwMap;
        arm = new NewArm();
        claw = new ClawFSM();
        arm.init(hwMap);
        claw.init(hwMap);
//        desperation = new ScuffedAngle();
//        desperation.init((hwMap));

    }

//    public void shift(double x, double y) {
//        arm.shift(x, y);
//    }

    public void update(){
        arm.PIDUpdate();
    }
}
