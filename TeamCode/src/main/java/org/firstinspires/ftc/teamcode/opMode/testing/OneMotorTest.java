package org.firstinspires.ftc.teamcode.opMode.testing;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name = "OneMotorTest")
public class OneMotorTest extends LinearOpMode {
    DcMotor motor;
    public void runOpMode(){
        motor = hardwareMap.get(DcMotor.class, "test1");
        waitForStart();
        while(opModeIsActive()) {
            motor.setPower(gamepad1.left_stick_y);
        }
    }
}
