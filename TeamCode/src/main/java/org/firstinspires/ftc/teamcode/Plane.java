package org.firstinspires.ftc.teamcode;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Plane implements Mechanism {
    HardwareMap hwMap;
    DcMotorEx planeMotor;

    @Override
    public void init(HardwareMap hwMap) {
        this.hwMap = hwMap;
        planeMotor = hwMap.get(DcMotorEx.class, "planeMotor");
    }
    public void on(){
        planeMotor.setPower(1);

    }
    public void off(){
        planeMotor.setPower(0);

    }
}
