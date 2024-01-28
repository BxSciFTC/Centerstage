package org.firstinspires.ftc.teamcode.opMode.testing.armTesting;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.teamcode.hardware.Mechanism;

public class ArmPresetsSafe implements Mechanism {
    public NewArm2 arm;
    HardwareMap hwMap;

    //is q1 angle in the model
    public double shoulderAngle;
    public double elbowAngle;
    ElapsedTime timer;

    @Override
    public void init(HardwareMap hwMap) {
        this.hwMap = hwMap;
        arm = new NewArm2();
        arm.init(hwMap);
        timer = new ElapsedTime();
        presets = Presets.REST;
        prevPresets = Presets.REST;
    }

    public static double restq1 = 90 , restq2 = 90;
    public static double pickupq1 = 90 , pickupq2 = 90;
    public static double score1q1 = 90 , score1q2 = 90;
    public static double score2q1 = 90 , score2q2 = 90;

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
                } else {
                    arm.shoulderGoToAngle(restq1);
                    arm.elbowGoToAngle(restq2);
                }
                break;
            case PICKUP:
                if (timer.milliseconds() < 1000 && shoulderAngle > 60) {
                    arm.shoulderGoToAngle(90);
                    arm.elbowGoToAngle(120);
                }
                else if (timer.milliseconds() < 2000 && shoulderAngle > 30) {
                    arm.shoulderGoToAngle(45);
                    arm.elbowGoToAngle(180);
                } else {
                    arm.shoulderGoToAngle(pickupq1);
                    arm.elbowGoToAngle(pickupq2);
                }
                break;
            case SCORE1:
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
