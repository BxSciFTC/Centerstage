package org.firstinspires.ftc.teamcode.opMode.teleOp;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

@TeleOp(name = "AMOGUS")
public class testArmThing extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        DcMotorEx shoulder = hardwareMap.get(DcMotorEx.class, "shoulder");
        DcMotorEx elbow = hardwareMap.get(DcMotorEx.class, "elbow");
        //DO NOT RESET THE ENCODERS, WE WANT TO MAINTAIN POSITION
        shoulder.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        elbow.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        shoulder.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        elbow.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);

        waitForStart();
        while(opModeIsActive()) {

            shoulder.setPower(0.4*gamepad1.left_stick_y);
            elbow.setPower(0.4*gamepad1.right_stick_y);
        }
    }

}
