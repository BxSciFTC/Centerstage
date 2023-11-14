package org.firstinspires.ftc.teamcode.opMode.testing;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.control.PIDCoefficients;
import com.acmerobotics.roadrunner.control.PIDFController;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;

//13 ppl

@Config
@TeleOp(name = "DashboardDemo")
public class Demo extends OpMode {
    private PIDFController controller;
    public static double p = 0, i = 0, d = 0;
    public static double f = 0;

    public static double correction = 0;


    public static int target = 100;

    private final double ticks = 537.7;

    private DcMotorEx motor;

    @Override
    public void init() {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        motor = hardwareMap.get(DcMotorEx.class, "test1");

    }

    @Override
    public void loop() {
        PIDCoefficients coeff = new PIDCoefficients(p, i, d);
        controller = new PIDFController(coeff, 0, 0, 0 , (p, v) -> f);
        int pos = motor.getCurrentPosition();

        correction = controller.update(pos);
        motor.setPower(correction);


        telemetry.addData("pos: ", pos);
        telemetry.addData("target: ", target);
        telemetry.addData("power", correction);
        telemetry.update();
    }
}
