package org.firstinspires.ftc.teamcode.hardware;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Plane implements Mechanism {
    HardwareMap hwMap;
    DcMotorEx planeMotor;

    boolean toggle = false;
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
        planeMotor.setPower(0.8);
        toggle = true;
    }
    public void off(){
        planeMotor.setPower(0);
        toggle = false;
    }

    public void update(){
        switch (planeState) {
            case ON:
                if(toggle == false) {
                    on();
                }
                break;
            case OFF:
                if(toggle == true){
                    off();
                }
        }
    }
}
