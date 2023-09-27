package org.firstinspires.ftc.teamcode.hardware;

import com.acmerobotics.roadrunner.profile.MotionProfile;
import com.acmerobotics.roadrunner.profile.MotionProfileGenerator;
import com.acmerobotics.roadrunner.profile.MotionState;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.acmerobotics.roadrunner.control.PIDCoefficients;
import com.acmerobotics.roadrunner.control.PIDFController;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Lift implements Mechanism {
    HardwareMap hwMap;

    //absolute encoders will be plugged into same port as motors
    DcMotorEx shoulder;
    DcMotorEx elbow;

    //count per revolution of the absolute encoders
    public static final double CPR = 0000000;

    //encoder counts for when the shoulder is at 0 degrees, and the elbow at 180
    //basically the elbow is extended all the way horizontally
    public static final double shoulder0 = 00000;
    public static final double elbow180 = 00000;


    public static final double shoulderkP = 0;
    public static final double shoulderkI = 0;
    public static final double shoulderkD = 0;
    public static int MAX_VEL_SHOULDER = 60;
    public static int MAX_ACCEL_SHOULDER = 60;


    public static final double elbowkP = 0;
    public static final double elbowkI = 0;
    public static final double elbowkD = 0;
    public static int MAX_VEL_ELBOW = 60;
    public static int MAX_ACCEL_ELBOW = 60;

    PIDCoefficients shoulderPID = new PIDCoefficients(shoulderkP, shoulderkI, shoulderkD);
    // create the controller
    PIDFController shoulderController = new PIDFController(shoulderPID, 0, 0 ,0, (p, v) -> getShoulderFg(p, v));
    public static MotionProfile profileShoulder;
    ElapsedTime timeShoulder = new ElapsedTime();

    PIDCoefficients elbowPID = new PIDCoefficients(shoulderkP, shoulderkI, shoulderkD);
    // create the controller
    PIDFController elbowController = new PIDFController(elbowPID, 0, 0, 0, (p, v) -> getElbowFg(p, v));
    public static MotionProfile profileElbow;
    ElapsedTime timeElbow = new ElapsedTime();

    boolean isReached = false;

    @Override
    public void init(HardwareMap hwMap) {
        this.hwMap = hwMap;
        shoulder = hwMap.get(DcMotorEx.class, "m1");
        elbow = hwMap.get(DcMotorEx.class, "m2");
        //DO NOT RESET THE ENCODERS, WE WANT TO MAINTAIN POSITION
        shoulder.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        elbow.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        shoulder.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        elbow.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    public void shoulderGoToAngle(double angle) {
        double prevPos = shoulder.getCurrentPosition();
        if (angleToCountShoulder(angle) == prevPos) return;
        double target = angleToCountShoulder(angle);

        profileShoulder = MotionProfileGenerator.generateSimpleMotionProfile(
                new MotionState(prevPos, 0, 0),
                new MotionState(target, 0, 0),
                MAX_VEL_SHOULDER,
                MAX_ACCEL_SHOULDER
        );
        timeShoulder.reset();
    }

    public void elbowGoToAngle(double angle) {
        double prevPos = elbow.getCurrentPosition();
        if (angleToCountElbow(angle) == prevPos) return;
        double target = angleToCountShoulder(angle);

        profileShoulder = MotionProfileGenerator.generateSimpleMotionProfile(
                new MotionState(prevPos, 0, 0),
                new MotionState(target, 0, 0),
                MAX_VEL_ELBOW,
                MAX_ACCEL_ELBOW
        );
        timeElbow.reset();
    }

    public void PIDUpdateShoulder() {
        MotionState state = profileShoulder.get(timeShoulder.seconds());
        shoulderController.setTargetPosition(state.getX());
        shoulderController.setTargetVelocity(state.getV());
        shoulderController.setTargetAcceleration(state.getA());
        double power = shoulderController.update(shoulder.getCurrentPosition());
        shoulder.setPower(power);
    }

    public void PIDUpdateElbow() {
        MotionState state = profileElbow.get(timeElbow.seconds());
        elbowController.setTargetPosition(state.getX());
        elbowController.setTargetVelocity(state.getV());
        elbowController.setTargetAcceleration(state.getA());
        double power = elbowController.update(elbow.getCurrentPosition());
        elbow.setPower(power);
    }


    public double angleToCountShoulder(double angle) {
        double counts = (360/angle)/CPR;
        //TODO: may be negative sign
        return shoulder0 + counts;

    }

    public double angleToCountElbow(double angle) {
        double counts = (360/angle)/CPR;
        //TODO: may be negative sign
        return elbow180 + counts;
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

    //can choose whether of not to use provided inputs, doesn't matter
    public double getShoulderFg(double position, double velocity) {
        //TODO:
        return 0;
    }


    //can choose whether of not to use provided inputs, doesn't matter
    public double getElbowFg(double position, double velocity) {
        //TODO:
        return 0;
    }

}
