package org.firstinspires.ftc.teamcode.hardware;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.controller.PIDFController;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

import static java.lang.Math.cos;

@Config
public class NewArm implements Mechanism {
    HardwareMap hwMap;

    //absolute encoders will be plugged into same port as motors
    DcMotorEx shoulder;
    DcMotorEx elbow;

    //count per revolution of the absolute encoders
    public static final double CPR = 0000000;

    //encoder counts for when the shoulder is at 0 degrees, and the elbow at 180
    //basically the elbow is extended all the way horizontally
    public static double shoulder0 = 00000;
    public static double elbow180 = 00000;


    public static double shoulderFg = 0;
    public static double motorFg = 0;
    public static double elbowFg = 0;


    public static double shoulderkP = 0;
    public static double shoulderkI = 0;
    public static double shoulderkD = 0;


    public static double elbowkP = 0;
    public static double elbowkI = 0;
    public static double elbowkD = 0;

    //remember to keep updating feedforward
    PIDFController shoulderController = new PIDFController(shoulderkP, shoulderkI, shoulderkD, getShoulderT());

    PIDFController elbowController = new PIDFController(shoulderkP, shoulderkI, shoulderkD, getShoulderT());

    boolean isReached = false;

    int tolerance = 10;

    @Override
    public void init(HardwareMap hwMap) {
        this.hwMap = hwMap;
        shoulder = hwMap.get(DcMotorEx.class, "shoulder");
        elbow = hwMap.get(DcMotorEx.class, "elbow");
        //DO NOT RESET THE ENCODERS, WE WANT TO MAINTAIN POSITION
        shoulder.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        elbow.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        shoulder.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        elbow.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        shoulderController.setTolerance(tolerance);
        elbowController.setTolerance(tolerance);
    }

    public void shoulderGoToAngle(double angle) {
        int prevPos = shoulder.getCurrentPosition();
        if (angleToCountShoulder(angle) == prevPos) return;
        int target = angleToCountShoulder(angle);

        shoulder.setTargetPosition(target);
    }

    public void elbowGoToAngle(double angle) {
        int prevPos = elbow.getCurrentPosition();
        if (angleToCountElbow(angle) == prevPos) return;
        int target = angleToCountShoulder(angle);

        elbow.setTargetPosition(target);
    }

    public void PIDUpdate() {
        PIDUpdateShoulder();
        PIDUpdateElbow();
    }

    public void PIDUpdateShoulder() {
        shoulderController.setF(getShoulderT());
    }

    public void PIDUpdateElbow() {
        shoulderController.setF(getElbowT());
    }


    private int angleToCountShoulder(double angle) {
        double counts = (angle/360)*CPR;
        //TODO: may be negative sign
        return (int)(shoulder0 + counts);

    }

    private int angleToCountElbow(double angle) {
        double counts = (angle/360)*CPR;
        //TODO: may be negative sign
        return (int)(elbow180 + counts);
    }

    //gets currents angle of shoulder in degrees
    public double shoulderDegrees() {
        double count = shoulder.getCurrentPosition();
        count -= shoulder0; //gets amount of ticks from 0 degrees

        //TODO: ENCODER MAY BE RUNNING IN OPPOSITE DIRECTION AND WE NEED TO CHANGE SIGNS

        count /= CPR; //gets # of revolutions
        count *= 360; //gets # of degrees

        return count;
    }

    //gets current angle of elbow in degrees
    public double elbowDegrees() {
        double count = elbow.getCurrentPosition();
        count -= shoulder0; //gets amount of ticks from 0 degrees

        //TODO: ENCODER MAY BE RUNNING IN OPPOSITE DIRECTION AND WE NEED TO CHANGE SIGNS

        count /= CPR; //gets # of revolutions
        count *= 360; //gets # of degrees

        return count;
    }

    public double shoulderRadians() {
        return Math.toRadians(shoulderDegrees());
    }

    public double elbowRadians() {
        return Math.toRadians(elbowDegrees());
    }

    //can choose whether of not to use provided inputs, doesn't matter
    private double getShoulderT() {
        double shoulderDegrees = shoulderDegrees();
        double elbowDegrees = elbowDegrees();
        double shoulderRadians = shoulderRadians();
        double elbowRadians = elbowRadians();
        double angle3 = shoulderRadians-Math.PI;
        double angle4 = 90 - Math.PI - shoulderRadians - angle3;
        if (shoulderDegrees >= 0 && shoulderDegrees < 90) {
            return motorFg*(cos(shoulderRadians))*(RobotConstants.shoulderLen) +
                    (elbowFg*cos(angle3)) * RobotConstants.shoulderLen / cos(angle4) +
                    shoulderFg * cos(shoulderRadians) * (RobotConstants.shoulderLen / 2);
        } else if (shoulderDegrees >= 90 && shoulderDegrees <= 180) {
            return motorFg * cos(shoulderRadians) * RobotConstants.shoulderLen +
                    elbowFg * cos(shoulderRadians) * RobotConstants.shoulderLen +
                    shoulderFg * cos(shoulderRadians);
        }
        return 0;
    }


    //can choose whether of not to use provided inputs, doesn't matter
    private double getElbowT() {
        double shoulderDegrees = shoulderDegrees();
        double elbowDegrees = elbowDegrees();

        //simulates polar axis
        double angleFromDown = elbowDegrees - (90 - shoulderDegrees) - 90;
        return RobotConstants.elbowLen * elbowFg * cos(angleFromDown * Math.PI / 180);
    }

}
