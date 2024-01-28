package org.firstinspires.ftc.teamcode.opMode.testing;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

@Config
@TeleOp(name = "SingleServoTest")
public class SingleServoTest extends LinearOpMode {
    Servo servo;

    public static double val = 0;
    @Override
    public void runOpMode() throws InterruptedException {

        servo = hardwareMap.get(Servo.class, "hinge");
        waitForStart();
        while (opModeIsActive()) {
            servo.setPosition(val);
        }
    }
}
