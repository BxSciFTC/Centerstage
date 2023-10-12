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

public class viperslideBACKUP implements Mechanism {

    HardwareMap hwMap;

    DcMotorEx lift;

    //based on height of each part of the thing where you stack the pixels
    public static final double floor1 = 0;
    public static final double floor2 = 0;
    public static final double floor3 = 0;
    public static final double floor4 = 0;

    public static double target = 0;
    public static double prevPos = target;
    public static boolean reachedPos = false;
    public static double tolerance = 30;


    public static double kG = -0.2;
    public static double kP = 0.008;
    public static double kI = 0;
    public static double kD = 0;

    public static PIDCoefficients c = new PIDCoefficients(kP, kI, kD);
    //                                                                                  accounts for the force of gravity
    public static PIDFController controller = new PIDFController(c, 0, 0, 0, (position, velocity) -> kG);
    public static MotionProfile profile;
    ElapsedTime PIDTimer = new ElapsedTime();
    public static int MAX_VEL = 60;
    public static int MAX_ACCEL = 50;


    @Override
    public void init(HardwareMap hwMap) {
        this.hwMap = hwMap;
        lift = hwMap.get(DcMotorEx.class, "lift");
        lift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        lift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        lift.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }
    //TODO: Implement states in LiftFSM
    public void setTargetPos(double pos) {
        if (target == pos) return;
        else reachedPos = false;

        target = pos;

        profile = MotionProfileGenerator.generateSimpleMotionProfile(
                new MotionState(prevPos, 0, 0),
                new MotionState(target, 0, 0),
                MAX_VEL,
                MAX_ACCEL
        );
        PIDTimer.reset();
        prevPos = target;
    }

    public void PIDUpdate() {
        MotionState state = profile.get(PIDTimer.seconds());
        controller.setTargetPosition(state.getX());
        controller.setTargetVelocity(state.getV());
        controller.setTargetAcceleration(state.getA());
        double power = controller.update(lift.getCurrentPosition());
        lift.setPower(power);
    }
    public void setPower(double power) {
        lift.setPower(power);
    }

    public boolean targetReached() {
        return (Math.abs(lift.getCurrentPosition()-target) < tolerance)
                && prevPos == target
                && lift.getVelocity() < 0.2; //have to see this work with the weight and the stress it puts
    }
}