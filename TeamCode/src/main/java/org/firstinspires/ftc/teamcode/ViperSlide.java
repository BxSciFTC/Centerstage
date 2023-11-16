package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.controller.PIDFController;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

@Config
public class ViperSlide implements Mechanism {
    HardwareMap hwMap;

    //absolute encoders will be plugged into same port as motors
    public DcMotorEx slide;

    //count per revolution of the absolute encoders
    public static final int CPR = 8192;

    public static int lowerLimit = 00000;
    public static int lowTick = 00000;
    public static int highTick = 00000;
    public static int upperLimit = 00000;

    public static double kP = 0;
    public static double kI = 0;
    public static double kD = 0;
    public static double kF = 0;

    public static int tolerance = 10;

    public static int currentTick = 0;

    //remember to keep updating feedforward
    PIDFController Controller = new PIDFController(kP, kI, kD, kF);

    @Override
    public void init(HardwareMap hwMap) {
        this.hwMap = hwMap;
        slide = hwMap.get(DcMotorEx.class, "slide");

        //DO NOT RESET THE ENCODERS, WE WANT TO MAINTAIN POSITION
        slide.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        slide.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        Controller.setTolerance(tolerance);
    }

    public void shiftSlide(int amount) {
        currentTick = slide.getCurrentPosition();
        int target = currentTick + amount;
        if (target > upperLimit || target < lowerLimit) return;

        slide.setTargetPosition(target);
    }

    public void update() {
        currentTick = slide.getCurrentPosition();
    }

    public void low() {
        slide.setTargetPosition(lowerLimit);
    }

    public void mid() {
        slide.setTargetPosition(lowTick);
    }

    public void high() {
        slide.setTargetPosition(highTick);
    }
}
