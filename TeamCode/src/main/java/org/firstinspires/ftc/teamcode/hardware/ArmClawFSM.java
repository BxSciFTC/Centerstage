package org.firstinspires.ftc.teamcode.hardware;

import com.qualcomm.robotcore.hardware.HardwareMap;

public class ArmClawFSM implements Mechanism {
    HardwareMap hwMap;
    ArmMapper arm;
    ClawFSM claw;
    @Override
    public void init(HardwareMap hwMap) {
        this.hwMap = hwMap;
        arm = new ArmMapper();
        claw = new ClawFSM();
        arm.init(hwMap);
        claw.init(hwMap);

    }

    public void shift(double x, double y) {
        arm.shift(x, y);
    }

    public void openClaw() {
        claw.open();
    }
    public void closeClaw() {
        claw.close();
    }

    public void floorOpen() {
        arm.moveTo(100000000, 10000000);
        claw.open();
    }

    public void placeInBasket() {
        arm.moveTo(100000000, 10000000);
        arm.moveTo(100000000, 10000000);
        claw.open();
        arm.moveTo(100000000, 10000000);
    }

    public void takeOutofBasket() {
        arm.moveTo(100000000, 10000000);
        arm.moveTo(100000000, 10000000);
        claw.open();
        arm.moveTo(100000000, 10000000);
        claw.close();
        arm.moveTo(100000000, 10000000);
    }
    public void update(){
        arm.PIDUpdate();
    }
}
