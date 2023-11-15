package org.firstinspires.ftc.teamcode.hardware;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Plane implements Mechanism {
    HardwareMap hwMap;
    DcMotorEx planeMotor;


    public PlaneState planeState;

    public enum PlaneState {
        ON,
        OFF,
    }

    @Override
    public void init(HardwareMap hwMap) {

        this.hwMap = hwMap;
        planeMotor = hwMap.get(DcMotorEx.class, "planeMotor");
        planeState = planeState.OFF;
    }
    public void on(){
        planeMotor.setPower(1);

    }
    public void off(){
        planeMotor.setPower(0);

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
