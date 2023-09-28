package org.firstinspires.ftc.teamcode.hardware;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class Claw implements Mechanism {
    HardwareMap hwMap;
    Servo clawControler;
    Servo hingeControler;

    @Override
    public void init(HardwareMap hwMap) {
        this.hwMap = hwMap;
        clawControler = hwMap.get(Servo.class, "ClawM");
        hingeControler = hwMap.get(Servo.class, "Hinge");
    }
}

