package org.firstinspires.ftc.teamcode.opMode.testing;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

@Config
@TeleOp(name = "demodemo")
public class demo2 extends OpMode {
    DcMotor motor;
    public static double power = 1;
    @Override
    public void init() {
        motor = hardwareMap.get(DcMotor.class, "test1");
    }

    @Override
    public void loop() {
        motor.setPower(power);
    }
}
