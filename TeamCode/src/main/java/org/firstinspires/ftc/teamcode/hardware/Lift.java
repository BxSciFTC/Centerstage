package org.firstinspires.ftc.teamcode.hardware;

import com.acmerobotics.roadrunner.control.PIDCoefficients;
import com.acmerobotics.roadrunner.control.PIDFController;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Lift implements org.firstinspires.ftc.teamcode.Hardware.Mechanism {
    HardwareMap hwMap;
    DcMotorEx shoulder;
    DcMotorEx arm;


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
    PIDFController shoulderController = new PIDFController(shoulderPID, 0, 0 ,0, (position, velocity) -> shoulderkG);

    PIDCoefficients armPID = new PIDCoefficients(shoulderkP, shoulderkI, shoulderkD);
    // create the controller
    PIDFController armController = new PIDFController(armPID, 0, 0, 0, (position, velocity) -> armkG);

    boolean isReached = false;



    @Override
    public void init(HardwareMap hwMap) {
        this.hwMap = hwMap;
        shoulder = hwMap.get(DcMotorEx.class, "m1");
        arm = hwMap.get(DcMotorEx.class, "m2");
        shoulder.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        shoulder.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        arm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        arm.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }
}
