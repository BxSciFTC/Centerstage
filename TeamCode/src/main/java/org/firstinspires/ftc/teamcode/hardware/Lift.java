package org.firstinspires.ftc.teamcode.hardware;

import org.firstinspires.ftc.teamcode.Hardware.Mechanism;
import com.acmerobotics.roadrunner.control.PIDCoefficients;
import com.acmerobotics.roadrunner.control.PIDFController;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Lift implements Mechanism {
    HardwareMap hwMap;

    //absolute encoders will be plugged into same port as motors
    DcMotorEx shoulder;
    DcMotorEx arm;

    //count per revolution of the absolute encoders
    public static final double CPR = 0000000;

    //encoder counts for when the shoulder is at 0 degrees, and the arm at 180
    //basically the arm is extended all the way horizontally
    public static final double shoulder0 = 00000;
    public static final double arm180 = 00000;

    public static final double shoulderkG = 0;
    public static final double shoulderkP = 0;
    public static final double shoulderkI = 0;
    public static final double shoulderkD = 0;

    public static final double armkG = 0;
    public static final double armkP = 0;
    public static final double armkI = 0;
    public static final double armkD = 0;

    PIDCoefficients shoulderPID = new PIDCoefficients(shoulderkP, shoulderkI, shoulderkD);
    // create the controller
    PIDFController shoulderController = new PIDFController(shoulderPID, 0, 0 ,0, (p, v) -> getShoulderFg(p, v));

    PIDCoefficients armPID = new PIDCoefficients(shoulderkP, shoulderkI, shoulderkD);
    // create the controller
    PIDFController armController = new PIDFController(armPID, 0, 0, 0, (p, v) -> getArmFg(p, v));


    boolean isReached = false;

    @Override
    public void init(HardwareMap hwMap) {
        this.hwMap = hwMap;
        shoulder = hwMap.get(DcMotorEx.class, "m1");
        arm = hwMap.get(DcMotorEx.class, "m2");
        //DO NOT RESET THE ENCODERS, WE WANT TO MAINTAIN POSITION
        shoulder.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        arm.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
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

    //gets current angle of arm in degrees
    public double armDegrees() {
        double count = arm.getCurrentPosition();
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
    public double getArmFg(double position, double velocity) {
        //TODO:
        return 0;
    }

}
