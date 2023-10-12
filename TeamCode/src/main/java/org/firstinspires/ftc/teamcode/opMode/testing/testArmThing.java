package org.firstinspires.ftc.teamcode.opMode.testing;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

@TeleOp(name = "AMOGUS")
public class testArmThing extends LinearOpMode {

    double elbowAngle;
    double shoulderAngle;

    @Override
    public void runOpMode() throws InterruptedException {
        DcMotorEx shoulder = hardwareMap.get(DcMotorEx.class, "shoulder");
        DcMotorEx elbow = hardwareMap.get(DcMotorEx.class, "elbow");
        //DO NOT RESET THE ENCODERS, WE WANT TO MAINTAIN POSITION
        shoulder.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        elbow.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        shoulder.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        elbow.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);

        elbowAngle = elbow.getCurrentPosition();
        shoulderAngle = shoulder.getCurrentPosition();


        waitForStart();
        while(opModeIsActive()) {
            shoulderAngle += gamepad1.left_stick_y;
            elbowAngle += gamepad1.right_stick_y;

            shoulder.setPower(shoulderAngle < shoulder.getCurrentPosition() ? -0.2 : 0.2);
            elbow.setPower(elbowAngle < elbow.getCurrentPosition() ? -0.2 : 0.2);
        }
    }

}
