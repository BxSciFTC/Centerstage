package org.firstinspires.ftc.teamcode.hardware;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class Plane implements Mechanism {
    HardwareMap hwMap;
    Servo planeMotor;


    public PlaneState planeState;

    public enum PlaneState {
        ON,
        OFF,
    }

    @Override
    public void init(HardwareMap hwMap) {

        this.hwMap = hwMap;
        planeMotor = hwMap.get(Servo.class, "planeMotor");
        planeState = planeState.OFF;
    }
    public void on(){
        planeMotor.setPosition(0);

    }
    public void off(){
        planeMotor.setPosition(0.5);

    }
    public void Ontoggle(){
        planeState=PlaneState.ON;
    }

    public void update(){
        switch (planeState) {
            case ON:
                on();
                break;
            case OFF:
                off();
        }
    }
}
