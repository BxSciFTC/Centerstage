package org.firstinspires.ftc.teamcode.hardware;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class Claw implements Mechanism {
    HardwareMap hwMap;
    Servo clawControler;
    Servo hingeControler;

    public static double elbowDownAngle;

    public static double open = 0.5;
    public static double close = -0.5;

    boolean isOpen;
    @Override
    public void init(HardwareMap hwMap) {
        this.hwMap = hwMap;
        clawControler = hwMap.get(Servo.class, "Claw");
        hingeControler = hwMap.get(Servo.class, "Hinge");
        elbowDownAngle = ArmMapper.elbowDownAngle;
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
        if(elbowDownAngle >= 0){
            double angle1 = 90 - elbowDownAngle;
            double servoAngle = angle1;
            hingeControler.setPosition(servoAngle);
        }
    }

    public void zeroAngleConstant(){
        //if()
    }

}

