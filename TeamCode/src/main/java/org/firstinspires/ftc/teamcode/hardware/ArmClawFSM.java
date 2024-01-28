package org.firstinspires.ftc.teamcode.hardware;

import com.qualcomm.robotcore.hardware.HardwareMap;

public class ArmClawFSM implements Mechanism {
    HardwareMap hwMap;
    ArmMapper arm;
    ClawFSM claw;

//    ScuffedAngle desperation;
    @Override
    public void init(HardwareMap hwMap) {
        this.hwMap = hwMap;
        arm = new ArmMapper();
        claw = new ClawFSM();
        arm.init(hwMap);
        claw.init(hwMap);
//        desperation = new ScuffedAngle();
//        desperation.init((hwMap));

    }

    public void shift(double x, double y) {
        arm.shift(x, y);
    }

    public void openClaw() {
        claw.leftOpen();
    }
    public void closeClaw() {
        claw.leftClose();
    }

    public void floorOpen() {
        arm.moveTo(100000000, 10000000);
        claw.leftOpen();
    }

    public void placeInBasket() {
        arm.moveTo(100000000, 10000000);
        arm.moveTo(100000000, 10000000);
        claw.leftOpen();
        arm.moveTo(100000000, 10000000);
    }

    public void takeOutofBasket() {
        arm.moveTo(100000000, 10000000);
        arm.moveTo(100000000, 10000000);
        claw.leftOpen();
        arm.moveTo(100000000, 10000000);
        claw.leftClose();
        arm.moveTo(100000000, 10000000);
    }
    /*public void high(){
        desperation.high();
    }*/
    /*public void low(){
        desperation.low();
    }*/
//    public void drop(){
//        desperation.drop();
//    }
//    public void floor(){
//        desperation.floor();
//    }
    public void update(){
        arm.PIDUpdate();
    }
}
