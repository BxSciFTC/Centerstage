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
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.opMode.teleOp.TeleOpMain;

@Config
public class Lift implements Mechanism {

    HardwareMap hwMap;
    DcMotorEx lift;
    Servo liftServo;

    Servo stopServo;
    public static double upPos = 0.6, downPos = 0.3, openPos = 0.8, closePos = 0.4;

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

    ElapsedTime stopTimer = new ElapsedTime();

    public void update(){
        Telemetry tele = TeleOpMain.tele;
        tele.addData("doing", 'x');

        if (liftServoState != prevServoState) {
            timer.reset();
        }
        switch (liftState){
            case UP:
                lift.setPower(1);
                tele.addData("upLift", 'x');
                break;
            case DOWN:
                lift.setPower(-1);
                tele.addData("downlift", 'x');
                break;
            case NORMAL:
                lift.setPower(0);
                tele.addData("normallift", 'x');
                break;
        }

        if (prevServoState != liftServoState) {
            stopTimer.reset();
        }

        switch (liftServoState) {
            case UP:
                tele.addData("upstop", 'x');
                if (stopTimer.milliseconds() < 1000) {
                    liftServo.setPosition(upPos);
                    tele.addData("upuplift", 'x');
                } else {
                    stopServo.setPosition(closePos);
                    tele.addData("downdownlift", 'x');
                }
                break;
            case DOWN:
                tele.addData("downstop", 'x');
                timer.reset();
                if (stopTimer.milliseconds() < 1000) {
                    stopServo.setPosition(openPos);
                    tele.addData("upupstop", 'x');
                } else {
                    liftServo.setPosition(downPos);
                    tele.addData("downdownstop", 'x');
                }
                break;
        }
        prevServoState = liftServoState;
        tele.update();
    }


}