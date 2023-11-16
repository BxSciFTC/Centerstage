package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

@Config
public class Claw implements Mechanism {
    HardwareMap hwMap;
    Servo clawController;
    Servo hingeController;

    public static double open = 0.5;
    public static double close = -0.5;

    public static double zeroDeg = 0000;
    public static double sixtyDeg = 0000;

    boolean isOpen = false;
    @Override
    public void init(HardwareMap hwMap) {
        this.hwMap = hwMap;
        clawController = hwMap.get(Servo.class, "claw");
        hingeController = hwMap.get(Servo.class, "hinge");
    }
    public void open() {
        clawController.setPosition(open);
        isOpen = true;
    }

    public void close() {
        clawController.setPosition(close);
        isOpen = false;
    }

    public void zeroDeg() {
        hingeController.setPosition(zeroDeg);
    }

    public void sixtyDeg() {
        hingeController.setPosition(sixtyDeg);
    }
}


