package org.firstinspires.ftc.teamcode.hardware;

import com.acmerobotics.roadrunner.control.PIDCoefficients;
import com.acmerobotics.roadrunner.control.PIDFController;
import com.acmerobotics.roadrunner.profile.MotionProfile;
import com.acmerobotics.roadrunner.profile.MotionProfileGenerator;
import com.acmerobotics.roadrunner.profile.MotionState;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

public class Lift implements Mechanism {

    HardwareMap hwMap;
    DcMotorEx lift;

    LiftState liftState;

    public enum LiftState {
        UP,
        DOWN,
        NORMAL, //meaning no movement;
    }
    @Override
    public void init(HardwareMap hwMap) {
        this.hwMap = hwMap;
        lift = hwMap.get(DcMotorEx.class, "lift");
        lift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        lift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        lift.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        liftState = LiftState.NORMAL;
    }

    public void setPower(double power) {
        lift.setPower(power);
    }

    public void update(){
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

    }


}