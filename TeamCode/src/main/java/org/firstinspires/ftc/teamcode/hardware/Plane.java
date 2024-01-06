package org.firstinspires.ftc.teamcode.hardware;
import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

@Config
public class Plane implements Mechanism {
    HardwareMap hwMap;
    Servo planeServo;
    ElapsedTime timer;


    public static double restore = 0.6;
    public static double release = 1;

    @Override
    public void init(HardwareMap hwMap) {
        timer = new ElapsedTime();
        this.hwMap = hwMap;
        planeServo = hwMap.get(Servo.class, "plane");
//        planeState = planeState.OFF;
    }

    public void fire() {
        timer.reset();
        planeServo.setPosition(release);
    }

    public void update() {
        if (timer.milliseconds() > 1200) {
            planeServo.setPosition(restore);
        }
    }

//    public PlaneState planeState;
//
//    public enum PlaneState {
//        ON,
//        OFF,
//    }
//
//    public void on(){
//        planeMotor.setPosition(0);
//
//    }
//    public void off(){
//        planeMotor.setPosition(0.5);
//
//    }
//    public void Ontoggle(){
//        planeState=PlaneState.ON;
//    }
//
//    public void update(){
//        switch (planeState) {
//            case ON:
//                on();
//                break;
//            case OFF:
//                off();
//        }
//    }
}
