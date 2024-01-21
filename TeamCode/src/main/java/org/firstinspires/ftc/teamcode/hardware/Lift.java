package org.firstinspires.ftc.teamcode.hardware;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.control.PIDCoefficients;
import com.acmerobotics.roadrunner.control.PIDFController;
import com.acmerobotics.roadrunner.profile.MotionProfile;
import com.acmerobotics.roadrunner.profile.MotionProfileGenerator;
import com.acmerobotics.roadrunner.profile.MotionState;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

@Config
public class Lift implements Mechanism {

    HardwareMap hwMap;
    DcMotorEx lift;
    Servo liftServo;

    Servo stopServo;
    public static double upPos = 0, downPos = 1, openPos = 0, closePos = 1;

    LiftState liftState;

    ElapsedTime timer;

    public enum LiftState {
        UP,
        DOWN,
        NORMAL, //meaning no movement;
    }

    LiftServoState liftServoState;

    public enum LiftServoState {
        UP,
        DOWN,
    }

    LiftServoState prevServoState;

    @Override
    public void init(HardwareMap hwMap) {
        this.hwMap = hwMap;
        lift = hwMap.get(DcMotorEx.class, "lift");
        liftServo = hwMap.get(Servo.class, "liftServo");
        stopServo = hwMap.get(Servo.class, "stopServo");
        lift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        lift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        lift.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        liftState = LiftState.NORMAL;
        liftServoState = LiftServoState.UP;
        prevServoState = LiftServoState.UP;
        timer = new ElapsedTime();
    }

    public void setPower(double power) {
        lift.setPower(power);
    }

    public void update(){
        if (liftServoState != prevServoState) {
            timer.reset();
        }
        switch (liftState){
            case UP:
                lift.setPower(1);

                break;
            case DOWN:
                lift.setPower(-1);

                break;
            case NORMAL:
                lift.setPower(0);
        }
        switch (liftServoState) {
            case UP:
                liftServo.setPosition(upPos);
                while (timer.milliseconds() <= 1000) {}
                stopServo.setPosition(closePos);
                break;
            case DOWN:
                timer.reset();
                stopServo.setPosition(openPos);
                while (timer.milliseconds() <= 1000) {}
                liftServo.setPosition(downPos);
                break;
        }
    }


}