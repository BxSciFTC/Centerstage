package org.firstinspires.ftc.teamcode.opMode.testing.armTesting;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.controller.PIDFController;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.TouchSensor;
import org.firstinspires.ftc.teamcode.hardware.Mechanism;
import org.firstinspires.ftc.teamcode.hardware.RobotConstants;

import static java.lang.Math.cos;

@Config
public class NewArm implements Mechanism {
    HardwareMap hwMap;

    public DcMotor shoulder;
    public DcMotor elbow;

    TouchSensor shoulderTouch;
    TouchSensor elbowTouch;

    //count per revolution of the absolute encoders
    public static final double CPR = 8192;

    //encoder counts for when the shoulder is at 0 degrees, and the elbow at 180
    //basically the elbow is extended all the way horizontally
    public static double shoulder0 = 00000;
    public static double elbow180 = 00000;


    public static double shoulderFg = 0.1;
    public static double motorFg = 0.1;
    public static double elbowFg = 0.1;


    public static double shoulderkP = 0.1;
    public static double shoulderkI = 0;
    public static double shoulderkD = 0;


    public static double elbowkP = 0.1;
    public static double elbowkI = 0;
    public static double elbowkD = 0;

    //remember to keep updating feedforward
    PIDFController shoulderController = new PIDFController(shoulderkP, shoulderkI, shoulderkD, 0); //placeholder

    PIDFController elbowController = new PIDFController(elbowkP, elbowkI, elbowkD, 0);

    boolean isReached = false;

//    int tolerance = 10;

    @Override
    public void init(HardwareMap hwMap) {
        this.hwMap = hwMap;
        shoulder = hwMap.get(DcMotor.class, "shoulder");
        elbow = hwMap.get(DcMotor.class, "elbow");
        shoulder.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        elbow.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        shoulder.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        elbow.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//        shoulderController.setTolerance(tolerance);
//        elbowController.setTolerance(tolerance);

        shoulderTouch = hwMap.get(TouchSensor.class, "ShoulderTouch");
        elbowTouch = hwMap.get(TouchSensor.class, "ElbowTouch");
    }

    public void shoulderGoToAngle(double angle) {
        int prevPos = shoulder.getCurrentPosition();
//        if (angleToCountShoulder(angle) == prevPos) return;
//        int target = angleToCountShoulder(angle);
        double power = shoulderController.calculate(prevPos, angle);

        shoulder.setTargetPosition((int)power);
    }

    public void elbowGoToAngle(double angle) {
        int prevPos = elbow.getCurrentPosition();
//        if (angleToCountElbow(angle) == prevPos) return;
//        int target = angleToCountShoulder(angle);
        double power = elbowController.calculate(prevPos, angle);

        elbow.setTargetPosition((int)power);
    }

    public void PIDUpdate() {
//        PIDUpdateShoulder();
//        PIDUpdateElbow();
        if (shoulderTouch.isPressed()) {
            calibrateShoulder();
        }
        if (elbowTouch.isPressed()) {
            calibrateElbow();
        }
        shoulderController.setP(shoulderkP);
        shoulderController.setI(shoulderkI);
        shoulderController.setD(shoulderkD);
        elbowController.setP(elbowkP);
        elbowController.setI(elbowkI);
        elbowController.setD(elbowkD);
    }

//    public void PIDUpdateShoulder() {
//        shoulderController.setF(getShoulderT());
//    }
//
//    public void PIDUpdateElbow() {
//        shoulderController.setF(getElbowT());
//    }
//
//    public static int negative1 = 1;
//    private int angleToCountShoulder(double angle) {
//        double counts = (angle/360)*CPR;
//        //TODO: may be negative sign
//        return negative1*(int)(shoulder0 + counts);
//
//    }
//
//    public static int negative2 = 1;
//    private int angleToCountElbow(double angle) {
//        double counts = (angle/360)*CPR;
//        //TODO: may be negative sign
//        return negative2*(int)(elbow180 + counts);
//    }
//
//    //gets currents angle of shoulder in degrees
//    public double shoulderDegrees() {
//        double count = shoulder.getCurrentPosition();
//        count -= shoulder0; //gets amount of ticks from 0 degrees
//
//        //TODO: ENCODER MAY BE RUNNING IN OPPOSITE DIRECTION AND WE NEED TO CHANGE SIGNS
//
//        count /= CPR; //gets # of revolutions
//        count *= 360; //gets # of degrees
//
//        return count;
//    }
//
//    //gets current angle of elbow in degrees
//    public double elbowDegrees() {
//        double count = elbow.getCurrentPosition();
//        count -= elbow180; //gets amount of ticks from 0 degrees
//
//        //TODO: ENCODER MAY BE RUNNING IN OPPOSITE DIRECTION AND WE NEED TO CHANGE SIGNS
//
//        count /= CPR; //gets # of revolutions
//        count *= 360; //gets # of degrees
//
//        return count;
//    }
//
//    public double shoulderRadians() {
//        return Math.toRadians(shoulderDegrees());
//    }
//
//    public double elbowRadians() {
//        return Math.toRadians(elbowDegrees());
//    }
//
//    //can choose whether of not to use provided inputs, doesn't matter
//    public double getShoulderT() {
//        double shoulderDegrees = shoulderDegrees();
//        double elbowDegrees = elbowDegrees();
//        double shoulderRadians = shoulderRadians();
//        double elbowRadians = elbowRadians();
//        double angle3 = shoulderRadians-Math.PI;
//        double angle4 = 90 - Math.PI - shoulderRadians - angle3;
//        if (shoulderDegrees >= 0 && shoulderDegrees < 90) {
//            return motorFg*(cos(shoulderRadians))*(RobotConstants.shoulderLen) +
//                    (elbowFg*cos(angle3)) * RobotConstants.shoulderLen / cos(angle4) +
//                    shoulderFg * cos(shoulderRadians) * (RobotConstants.shoulderLen / 2);
//        } else if (shoulderDegrees >= 90 && shoulderDegrees <= 180) {
//            return motorFg * cos(shoulderRadians) * RobotConstants.shoulderLen +
//                    elbowFg * cos(shoulderRadians) * RobotConstants.shoulderLen +
//                    shoulderFg * cos(shoulderRadians);
//        }
//        return 0;
//    }
//
//
//    //can choose whether of not to use provided inputs, doesn't matter
//    public double getElbowT() {
//        double shoulderDegrees = shoulderDegrees();
//        double elbowDegrees = elbowDegrees();
//
//        //simulates polar axis
//        double angleFromDown = elbowDegrees - (90 - shoulderDegrees) - 90;
//        return RobotConstants.elbowLen * elbowFg * cos(angleFromDown * Math.PI / 180);
//    }

    public void calibrateShoulder() {
        shoulder.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }

    public void calibrateElbow() {
        elbow.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }
}
