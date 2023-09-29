package org.firstinspires.ftc.teamcode.lastYearCode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.teamcode.lastYearCode.Controls;

@TeleOp (name="drivenoi", group = "_1")
public class TeleOpMain extends LinearOpMode {
    Controls bot = new Controls();

    public void runOpMode(){
        bot.init(hardwareMap);
        telemetry.addData("TeleOpMain", "boop1");
        telemetry.update();
        waitForStart();
        while(opModeIsActive()) {
            bot.run(gamepad1);
        }
        telemetry.addData("TeleOpMain", "boop2");
        telemetry.update();
    }
}