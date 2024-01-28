package org.firstinspires.ftc.teamcode.opMode.testing;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.DcMotor;

//@Config
//@TeleOp(name = "EncoderTest")
//public class EncoderTest extends OpMode {
//
//    AnalogInput input;
//
//
//    //pog
//    public static int value = 0;
//
//    @Override
//    public void init() {
//        input = hardwareMap.get(AnalogInput.class, "encoder1");
//        input.
//        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
//    }
//
//    @Override
//    public void loop() {
//        value = motor.getCurrentPosition();
//        telemetry.addData("Encoder", value);
//        telemetry.update();
//    }
//}
//

@Disabled
@Config
@TeleOp(name = "EncoderTest")

public class EncoderTest extends OpMode {

    DcMotor motor;
//pog
    public static int value = 0;

    @Override
    public void init() {
        motor = hardwareMap.get(DcMotor.class, "test1");
        motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
    }

    @Override
    public void loop() {
        value = motor.getCurrentPosition();
        telemetry.addData("Encoder", value);
        telemetry.update();
    }
}
