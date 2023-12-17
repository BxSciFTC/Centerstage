package org.firstinspires.ftc.teamcode.opMode.testing.armTesting;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOp(name = "help")
public class pain extends LinearOpMode {
    DcMotor motor;
    @Override
    public void runOpMode() {
        motor = hardwareMap.get(DcMotor.class, "shoulder");

        waitForStart();
        while (opModeIsActive()) {
            motor.setPower(gamepad1.left_stick_y);
        }
    }
}
