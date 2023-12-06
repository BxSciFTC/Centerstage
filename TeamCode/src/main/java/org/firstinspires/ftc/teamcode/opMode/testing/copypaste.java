package org.firstinspires.ftc.teamcode.opMode.testing;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.control.PIDFController;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import org.firstinspires.ftc.teamcode.hardware.Controller;

@Config
@TeleOp(name = "copypaste")
public class copypaste extends OpMode {
    private PIDController controller;
    public static double p = 1, i = 0, d = 0, f = 0;
    public static int target = 0;

    private DcMotorEx motor;

    @Override
    public void init() {
        controller = new PIDController(0,0,0);
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        motor = hardwareMap.get(DcMotorEx.class, "test1");

    }

    @Override
    public void loop() {
        controller.setPID(p, i, d);
        int armPos = motor.getCurrentPosition();
        double pid  = controller.calculate(armPos, target);

        double power = pid + f;
        motor.setPower(power);

        telemetry.addData("pos", armPos);
        telemetry.addData("target", target);
        telemetry.update();
    }
}
