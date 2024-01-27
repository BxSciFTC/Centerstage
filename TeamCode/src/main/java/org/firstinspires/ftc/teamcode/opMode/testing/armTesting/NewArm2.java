package org.firstinspires.ftc.teamcode.opMode.testing.armTesting;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.TouchSensor;
import org.firstinspires.ftc.teamcode.hardware.Mechanism;
import org.firstinspires.ftc.teamcode.hardware.RobotConstants;

import static java.lang.Math.*;


@Config
public class NewArm2 implements Mechanism {
    HardwareMap hwMap;
    public static DcMotorEx shoulder;
    public static DcMotorEx shoulder2;
    public static DcMotorEx elbow;
    TouchSensor shoulderTouch;
    boolean prevShoulderTouch;
    TouchSensor elbowTouch;
    boolean prevElbowTouch;

    private PIDController shoulderController;
    public static double p1 = 0.03, i1 = 0, d1 = 0, ff1 = -0.5;

    private PIDController elbowController;
    public static double p2 = -0.03, i2 = -0.0, d2 = -0.00, ff2 = 0.7;


    //count per revolution of the absolute encoders
    public static final double CPR = 537.7;

    //encoder counts for when the shoulder is at 0 degrees, and the elbow at 180
    //basically the elbow is extended all the way horizontally

    //default 185 degrees
    public static int shoulder185 = 264;

    //default reset 11 degrees
    public static int elbow11 = 39;

    public static double shoulderPower = 1;

    public static double elbowPower = 1;

    public void powerOff() {
        shoulderPower = 0;
        elbowPower = 0;
    }

    public void powerOn() {
        shoulderPower = 1;
        elbowPower = 1;
    }

    public void shoulderGoToAngle(double angle) {
        shoulderTarget = angleToCountShoulder(angle);
    }

    public void elbowGoToAngle(double angle) {
        elbowTarget = angleToCountElbow(angle);
    }

//    public static int negative1 = 1;

    public static int angleToCountShoulder(double angle) {
        double counts = (angle / 360) * CPR;

        counts = shoulder185 - counts;

        //TODO: may be negative sign
        return (int) (counts);
    }


    public static int angleToCountElbow(double angle) {
        double counts = (angle / 360) * CPR;

        counts = -elbow11 + counts - shoulder.getCurrentPosition();

        //TODO: may be negative sign
        return (int) (counts);
    }

    //gets currents angle of shoulder in degrees
    public double shoulderDegrees() {
        double count = shoulder.getCurrentPosition();
        count = shoulder185 - count;

        //TODO: ENCODER MAY BE RUNNING IN OPPOSITE DIRECTION AND WE NEED TO CHANGE SIGNS

        count /= CPR; //gets # of revolutions
        count *= 360; //gets # of degrees

        return count;
    }

    //gets current angle of elbow in degrees
    public double elbowDegrees() {
        double count = elbow.getCurrentPosition();

        count = elbow11 + count + shoulder.getCurrentPosition();
        //easter egg
        //TODO: ENCODER MAY BE RUNNING IN OPPOSITE DIRECTION AND WE NEED TO CHANGE SIGNS

        count /= CPR; //gets # of revolutions
        count *= 360; //gets # of degrees

        return count;
    }


    public static int shoulderTarget = shoulder185;
    public static int elbowTarget = elbow11;
    public static int shoulderPos = 0;
    public static int elbowPos = 0;


    @Override
    public void init(HardwareMap hwMap) {
        this.hwMap = hwMap;
        shoulderController = new PIDController(p1, i1, d1);
        elbowController = new PIDController(p2, i2, d2);

        shoulder = hwMap.get(DcMotorEx.class, "shoulder");
        shoulder2 = hwMap.get(DcMotorEx.class, "shoulder2");
        elbow = hwMap.get(DcMotorEx.class, "elbow");

        shoulderTouch = hwMap.get(TouchSensor.class, "shoulderTouch");
        elbowTouch = hwMap.get(TouchSensor.class, "elbowTouch");

        prevShoulderTouch = false;
        prevElbowTouch = false;
    }

    public void PIDUpdate() {
        shoulderController.setPID(p1, i1, d1);
        elbowController.setPID(p2, i2, d2);

        shoulderPos = shoulder.getCurrentPosition();
        elbowPos = elbow.getCurrentPosition();

        double powerShoulder = shoulderController.calculate(shoulderPos, shoulderTarget);
        shoulder.setPower(shoulderPower*(powerShoulder + shoulderFF()));
        shoulder2.setPower(shoulderPower*(powerShoulder + shoulderFF()));

        double powerElbow = elbowController.calculate(elbowPos, elbowTarget);
        elbow.setPower(elbowPower*(-1 * powerElbow + elbowFF()));


        //FIX ELBOW CALIBRATE IS INCORRECT WHILE SHOULDER IS NOT ZERO
        if (shoulderTouch.isPressed()) {
            calibrateShoulder();
        } else {
            prevShoulderTouch = false;
        }
        if (shoulderTouch.isPressed() && elbowTouch.isPressed()) {
            calibrateElbow();
        } else {
            prevElbowTouch = false;
        }
    }

//    public static double motorFg = 0;
//    public static double elbowFg = 0;
//    public static double shoulderFg = 0;

    public static double weight1 = 1;
    public static double weight2 = 0.6;
    public static double COMAngle = 0;

    public double shoulderFF() {
//        double downAngle = shoulderDegrees();
//        return ff1 * cos(Math.toRadians(downAngle));
        double shoulderAngle = Math.toRadians(shoulderDegrees());
        double downAngle = Math.toRadians(shoulderDegrees() + elbowDegrees() - 180);

        //shoulder COM
        double x1 = RobotConstants.shoulderLen * cos(shoulderAngle) / 2;
        double y1 = RobotConstants.shoulderLen * sin(shoulderAngle) / 2;

        //elbow COM
        double x2 = (2 * x1) + (RobotConstants.elbowLen * cos(downAngle) / 2);
        double y2 = (2 * y1) + (RobotConstants.elbowLen * sin(downAngle) / 2);

        double COMx = (weight1 * x1 + weight2 * x2) / 2;
        double COMy = (weight1 * y1 + weight2 * y2) / 2;

        COMAngle = atan2(COMy,COMx);

        return ff1 * cos(COMAngle);

//        double shoulderDegrees = shoulderDegrees();
//        double elbowDegrees = elbowDegrees();
//        double shoulderRadians = Math.toRadians(shoulderDegrees);
//        double elbowRadians = Math.toRadians(elbowDegrees);
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
    }

    public static double downdownAngle = 0;

    public double elbowFF() {
        double downAngle = shoulderDegrees() + elbowDegrees() - 180;
        downdownAngle = downAngle;
        return ff2 * cos(Math.toRadians(downAngle));
    }

    public void calibrateShoulder() {
        if (prevShoulderTouch == false) {
            shoulder.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            shoulder.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            prevShoulderTouch = true;
        } else
            return;
    }

    public void calibrateElbow() {
        if (prevElbowTouch == false) {
            elbow.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            elbow.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            prevElbowTouch = true;
        } else
            return;
    }
}