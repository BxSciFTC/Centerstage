package org.firstinspires.ftc.teamcode.hardware;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

@Config
public class ArmPresetsSafe implements Mechanism {
    public NewArm arm;
    HardwareMap hwMap;

    //is q1 angle in the model
    public double shoulderAngle;
    public double elbowAngle;
    ElapsedTime timer;

    @Override
    public void init(HardwareMap hwMap) {
        this.hwMap = hwMap;
        arm = new NewArm();
        arm.init(hwMap);
        timer = new ElapsedTime();
        presets = Presets.REST;
        prevPresets = Presets.REST;
    }

    public static double restq1 = 180 , restq2 = 26;
    public static double pickupq1 = 0 , pickupq2 = 180;
    public static double score1q1 = 90 , score1q2 = 90;
    public static double score2q1 = 60 , score2q2 = 120;

    Presets presets;
    Presets prevPresets;

    public enum Presets {
        REST,
        PICKUP,
        SCORE1,
        SCORE2,
    }

    public void rest() {
        presets = Presets.REST;
    }

    public void pickup() {
        presets = Presets.PICKUP;
    }

    public void score1() {
        presets = Presets.SCORE1;
    }

    public void score2() {
        presets = Presets.SCORE2;
    }

    public void PIDUpdate() {
        arm.PIDUpdate();

        if (prevPresets != presets) {
            timer.reset();
        }

        shoulderAngle = arm.shoulderDegrees();
        elbowAngle = arm.elbowDegrees();

        switch (presets) {
            case REST:
                if (timer.milliseconds() < 2000 && shoulderAngle < 135) {
                    arm.shoulderGoToAngle(90);
                    arm.elbowGoToAngle(45);
                } else if (timer.milliseconds() < 3000) {
                    arm.shoulderGoToAngle(restq1);
                    arm.elbowGoToAngle(restq2);
                } else {
                    arm.powerOff();
                }
                break;
            case PICKUP:
                arm.powerOn();
                if (timer.milliseconds() < 1000 && shoulderAngle > 60) {
                    arm.shoulderGoToAngle(90);
                    arm.elbowGoToAngle(120);
                }
                else if (timer.milliseconds() < 2000 && shoulderAngle > 30) {
                    arm.shoulderGoToAngle(45);
                    arm.elbowGoToAngle(180);
                } else if (timer.milliseconds() < 3000) {
                    arm.shoulderGoToAngle(pickupq1);
                    arm.elbowGoToAngle(pickupq2);
                } else {
                    arm.powerOff();
                }
                //MAYBE POWER OFF IF NEEDED
                break;
            case SCORE1:
                arm.powerOn();
                if (timer.milliseconds() < 1000 && shoulderAngle > 60) {
                    arm.shoulderGoToAngle(90);
                    arm.elbowGoToAngle(120);
                }
                else {
                    arm.shoulderGoToAngle(score2q1);
                    arm.elbowGoToAngle(score2q2);
                }
                break;
            case SCORE2:
                arm.powerOn();
                if (timer.milliseconds() < 1000 && shoulderAngle > 60) {
                    arm.shoulderGoToAngle(90);
                    arm.elbowGoToAngle(120);
                } else {
                    arm.shoulderGoToAngle(score1q1);
                    arm.elbowGoToAngle(score1q2);
                }
                break;
        }
        prevPresets = presets;
    }
}
