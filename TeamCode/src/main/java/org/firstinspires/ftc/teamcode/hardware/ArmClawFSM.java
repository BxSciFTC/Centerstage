package org.firstinspires.ftc.teamcode.hardware;

import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.teamcode.opMode.teleOp.TeleOpMain;

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
        TeleOpMain.tele.addData("fsm111", "1");
    }
    public void rest() {
        claw.setRest();
        arm.rest();
        TeleOpMain.tele.addData("fsm222", "1");
    }
    public void pickup() {
        claw.setPickup();
        arm.pickup();
        TeleOpMain.tele.addData("fsm333", "1");
    }
    public void score1() {
        claw.setScore1();
        arm.score1();
        TeleOpMain.tele.addData("fsm444", "1");
    }
    public void score2() {
        claw.setScore2();
        arm.score2();
        TeleOpMain.tele.addData("fsm555", "1");
    }

    public void update(){
        arm.PIDUpdate();
    }
}
