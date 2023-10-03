package org.firstinspires.ftc.teamcode.OpMode.TeleOp;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.teamcode.hardware.Controller;

@TeleOp(name = "TeleOpMain")
public class TeleOpMain extends LinearOpMode {

    Controller bot = new Controller();

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
