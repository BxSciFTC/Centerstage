package org.firstinspires.ftc.teamcode.hardware;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class Claw implements Mechanism {
    HardwareMap hwMap;
    Servo clawControler;
    Servo hingeControler;

    public static double open = 0.5;
    public static double close = -0.5;

    @Override
    public void init(HardwareMap hwMap) {
        this.hwMap = hwMap;
        clawControler = hwMap.get(Servo.class, "Claw");
        hingeControler = hwMap.get(Servo.class, "Hinge");
    }
    public void open() {
        clawControler.setPosition(open);
    }
}

