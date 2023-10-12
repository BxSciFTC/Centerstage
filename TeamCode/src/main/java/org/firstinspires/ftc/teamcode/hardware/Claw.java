package org.firstinspires.ftc.teamcode.hardware;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class Claw implements Mechanism {
    HardwareMap hwMap;
    Servo clawControler;
    Servo hingeControler;

    public static double elbowAngle;
    public static double shoulderAngle;

    public static double open = 0.5;
    public static double close = -0.5;

    boolean isOpen;
    @Override
    public void init(HardwareMap hwMap) {
        this.hwMap = hwMap;
        clawControler = hwMap.get(Servo.class, "Claw");
        hingeControler = hwMap.get(Servo.class, "Hinge");
        elbowAngle = ArmMapper.elbowAngle;
        shoulderAngle = ArmMapper.shoulderAngle;
    }
    public void open() {
        clawControler.setPosition(open);
        isOpen = true;
    }

    public void close() {
        clawControler.setPosition(close);
        isOpen = false;
    }

    public void thirtyAngleConstant(){
        double elbowDownAngle = 180 - shoulderAngle - elbowAngle;
        double a = 90 - elbowDownAngle;
        double servoAngle = a + 30;
        hingeControler.setPosition(Func.map(servoAngle, -90, 180, -1, 1));
    }

    public void zeroAngleConstant(){
        double elbowDownAngle = 180 - shoulderAngle - elbowAngle;
        double a = 90 - elbowDownAngle;
        double servoAngle = a + 90;
        hingeControler.setPosition(Func.map(servoAngle, -90, 180, -1, 1));
    }

}


