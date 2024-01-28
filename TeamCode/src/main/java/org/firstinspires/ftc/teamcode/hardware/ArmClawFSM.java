package org.firstinspires.ftc.teamcode.hardware;

import com.qualcomm.robotcore.hardware.HardwareMap;

public class ArmClawFSM implements Mechanism {
    HardwareMap hwMap;
    ArmPresetsSafe arm;
    ClawFSM claw;

//    ScuffedAngle desperation;
    @Override
    public void init(HardwareMap hwMap) {
        this.hwMap = hwMap;
        arm = new ArmPresetsSafe();
        claw = new ClawFSM();
        arm.init(hwMap);
        claw.init(hwMap);
//        desperation = new ScuffedAngle();
//        desperation.init((hwMap));

    }

//    public void shift(double x, double y) {
//        arm.shift(x, y);
//    }

    public void preAutonRest() {
        claw.setPreAutonRest();
        arm.rest();
    }
    public void rest() {
        claw.setRest();
        arm.rest();
    }
    public void pickup() {
        claw.setPickup();
        arm.pickup();
    }
    public void score1() {
        claw.setScore1();
        arm.score1();
    }
    public void score2() {
        claw.setScore2();
        arm.score2();
    }

    public void update(){
        arm.PIDUpdate();
    }
}
