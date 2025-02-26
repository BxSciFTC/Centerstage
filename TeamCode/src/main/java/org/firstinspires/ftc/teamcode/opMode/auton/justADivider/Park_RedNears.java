package org.firstinspires.ftc.teamcode.opMode.auton.justADivider;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
@Disabled
@Autonomous(name = "leftfff")
public class Park_RedNears extends LinearOpMode {
    DcMotor leftFront = hardwareMap.get(DcMotorEx.class, "leftFront");
    DcMotor leftRear = hardwareMap.get(DcMotorEx.class, "leftRear");
    DcMotor rightFront = hardwareMap.get(DcMotorEx.class, "rightRear");
    DcMotor rightRear = hardwareMap.get(DcMotorEx.class, "rightFront");

    //ArmClawFSM deposit = new ArmClawFSM();
    @Override
    public void runOpMode() throws InterruptedException {

        leftRear.setDirection(DcMotor.Direction.REVERSE);
        waitForStart();

        leftFront.setPower(-0.5);
        leftRear.setPower(0.5);
        rightFront.setPower(0.5);
        rightRear.setPower(-0.5);
        wait(3000);

        while (opModeIsActive() && !isStopRequested()) {
            leftFront.setPower(0);
            leftRear.setPower(0);
            rightFront.setPower(0);
            rightRear.setPower(0);
        }
    }
}

