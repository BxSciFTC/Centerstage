package org.firstinspires.ftc.teamcode.opMode.testing;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.teamcode.hardware.Claw;

@Config
@TeleOp(name = "Servo Test")
public class ServoTest extends OpMode {
    Claw claw;
    @Override
    public void init() {
        claw = new Claw();
        claw.init(hardwareMap);
    }

    public static double run = 1;

    @Override
    public void loop() {
        if (run == 1) {
            claw.open();
        } else {
            claw.close();
        }
    }
}
