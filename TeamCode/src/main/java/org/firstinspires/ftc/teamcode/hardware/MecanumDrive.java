package org.firstinspires.ftc.teamcode.hardware;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

@Config
public class MecanumDrive implements Mechanism {
    private HardwareMap hwMap;

    private DcMotorEx leftFront;
    private DcMotorEx leftRear;
    private DcMotorEx rightFront;
    private DcMotorEx rightRear;

    public void init(HardwareMap hwMap) {
        this.hwMap = hwMap;
        init();
    }

    private void init() {
        leftFront = hwMap.get(DcMotorEx.class, "leftFront");
        leftRear = hwMap.get(DcMotorEx.class, "leftRear");
        rightRear = hwMap.get(DcMotorEx.class, "rightRear");
        rightFront = hwMap.get(DcMotorEx.class, "rightFront");

        leftFront.setDirection(DcMotor.Direction.REVERSE);
        leftRear.setDirection(DcMotor.Direction.REVERSE);

        leftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftRear.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightRear.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

    }

    //used to fix imperfect strafing
    //TODO: test this value with the driver
    public static double STRAFE_INCREASE = 1.1;
    //disregard the pose header, only use magnitude
    public void setWeightedDrivePower(Gamepad gamepad) {
        double x = gamepad.left_stick_x * STRAFE_INCREASE;

        //may or may not need to flip this sign - +
        double y = -gamepad.left_stick_y;

        //right stick controls turning
        double rx = gamepad.right_stick_x;

        //typical Mecanum Drive power calculations
        double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);
        leftFront.setPower((y + x + rx) /denominator);
        leftRear.setPower((y - x + rx)  /denominator);
        rightFront.setPower((y - x - rx)/denominator);
        rightRear.setPower((y + x - rx) /denominator);
    }
}
