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

import static java.lang.Math.cos;


@Config
public class NewArm2 implements Mechanism {
    HardwareMap hwMap;
    DcMotorEx shoulder;
    DcMotorEx elbow;
    TouchSensor shoulderTouch;
    boolean prevShoulderTouch;
    TouchSensor elbowTouch;
    boolean prevElbowTouch;

    private PIDController shoulderController;
    public static double p1 = 0.01, i1 = 0, d1 = 0;

    private PIDController elbowController;
    public static double p2 = 0.01, i2 = 0, d2 = 0;


    //count per revolution of the absolute encoders
    public static final double CPR = 537.7;

    //encoder counts for when the shoulder is at 0 degrees, and the elbow at 180
    //basically the elbow is extended all the way horizontally

    //default 185 degrees
    public static int shoulder185 = 00000;

    //default reset 11 degrees
    public static int elbow11 = 00000;


    public void shoulderGoToAngle(double angle) {
        shoulderPos = shoulder.getCurrentPosition();
        if (angleToCountShoulder(angle) == shoulderPos) return;
        shoulderTarget = angleToCountShoulder(angle);


    }

    public void elbowGoToAngle(double angle) {
        elbowPos = elbow.getCurrentPosition();
        if (angleToCountElbow(angle) == elbowPos) return;
        elbowTarget = angleToCountElbow(angle);
    }

    public static int negative1 = 1;

    private int angleToCountShoulder(double angle) {
        double counts = (angle / 360) * CPR;
        //TODO: may be negative sign
        return negative1 * (int) (shoulder185 + counts);

    }

    public static int negative2 = 1;

    private int angleToCountElbow(double angle) {
        double counts = (angle / 360) * CPR;
        //TODO: may be negative sign
        return negative2 * (int) (elbow11 + counts);
    }

    //gets currents angle of shoulder in degrees
    public double shoulderDegrees() {
        double count = shoulder.getCurrentPosition();
        count -= shoulder185; //gets amount of ticks from 0 degrees

        //TODO: ENCODER MAY BE RUNNING IN OPPOSITE DIRECTION AND WE NEED TO CHANGE SIGNS

        count /= CPR; //gets # of revolutions
        count *= 360; //gets # of degrees

        return count;
    }

    //gets current angle of elbow in degrees
    public double elbowDegrees() {
        double count = elbow.getCurrentPosition();
        count -= elbow11; //gets amount of ticks from 0 degrees

        //TODO: ENCODER MAY BE RUNNING IN OPPOSITE DIRECTION AND WE NEED TO CHANGE SIGNS

        count /= CPR; //gets # of revolutions
        count *= 360; //gets # of degrees

        return count;
    }


    public static int shoulderTarget = 0;
    public static int elbowTarget = 0;
    public int shoulderPos = 0;
    public int elbowPos = 0;

    public double power1 = 0;
    public double power2 = 0;


    @Override
    public void init(HardwareMap hwMap) {
        this.hwMap = hwMap;
        shoulderController = new PIDController(p1, i1, d1);
        elbowController = new PIDController(p2, i2, d2);

        shoulder = hwMap.get(DcMotorEx.class, "shoulder");
        elbow = hwMap.get(DcMotorEx.class, "elbow");

        shoulderTouch = hwMap.get(TouchSensor.class, "shoulderTouch");
        elbowTouch = hwMap.get(TouchSensor.class, "elbowTouch");

        prevShoulderTouch = false;
        prevElbowTouch = false;
    }

    public void PIDUpdate() {
        shoulderController.setPID(p1, i1, d1);
        elbowController.setPID(p2, i2, d2);

        power1 = shoulderController.calculate(shoulderPos, shoulderTarget);
        shoulder.setPower(power1 + shoulderFF());

        power2 = elbowController.calculate(elbowPos, elbowTarget);
        elbow.setPower(power2 + elbowFF());

        if (shoulderTouch.isPressed()) {
            calibrateShoulder();
        } else {
            prevShoulderTouch = false;
        }
        if (elbowTouch.isPressed()) {
            calibrateElbow();
        } else {
            prevElbowTouch = false;
        }
    }

    public static double motorFg = 0;
    public static double elbowFg = 0;
    public static double shoulderFg = 0;

    public double shoulderFF() {
        return 0;
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

    public double elbowFF() {
        return 0;
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