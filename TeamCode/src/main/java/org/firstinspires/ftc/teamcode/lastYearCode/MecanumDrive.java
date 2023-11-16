package org.firstinspires.ftc.teamcode.lastYearCode;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

@Config
public class MecanumDrive {
    private HardwareMap hwMap;

    private DcMotorEx leftFront;
    private DcMotorEx leftRear;
    private DcMotorEx rightFront;
    private DcMotorEx rightRear;

    public MecanumDrive(HardwareMap hwMap) {
        this.hwMap = hwMap;
        init();
    }

    private void init() {
        leftFront = hwMap.get(DcMotorEx.class, "leftFront");
        leftRear = hwMap.get(DcMotorEx.class, "leftRear");
        rightFront = hwMap.get(DcMotorEx.class, "rightRear");
        rightRear = hwMap.get(DcMotorEx.class, "rightFront");

        leftFront.setDirection(DcMotor.Direction.REVERSE);
        leftRear.setDirection(DcMotor.Direction.REVERSE);

        leftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftRear.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightRear.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

    }

    //used to fix imperfect strafing
    //TODO: test this value with the driver
    double STRAFE_INCREASE = 1.1;
    public static int neg1 = 1;
    public static int neg2 = 1;
    public static int neg3 = 1;
    public static int neg4 = 1;
    public static int neg5 = 1;
    public static int neg6 = 1;
    public static int neg7 = 1;

    //disregard the pose header, only use magnitude
    public void setWeightedDrivePower(Gamepad gamepad) {
        double x = neg1 * gamepad.left_stick_x * STRAFE_INCREASE;

        //SOLUTION MAY BE SWITCH Y AND RX

        //may or may not need to flip this sign - +
        double y = neg2 * gamepad.left_stick_y;

        //right stick controls turning
        double rx = neg3 * gamepad.right_stick_x;

        //typical Mecanum Drive power calculations
        double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);
        leftFront.setPower(neg4*(y + x + rx) /denominator);
        leftRear.setPower(neg5*(y - x + rx)  /denominator);
        rightFront.setPower(neg6*(y - x - rx)/denominator);
        rightRear.setPower(neg7*(y + x - rx) /denominator);
    }
}
