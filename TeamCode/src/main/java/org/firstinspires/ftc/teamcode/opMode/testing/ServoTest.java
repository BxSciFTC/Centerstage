package org.firstinspires.ftc.teamcode.opMode.testing;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.teamcode.hardware.ClawFSM;

@Disabled
@Config
@TeleOp(name = "Servo Test")
public class ServoTest extends LinearOpMode {
    ClawFSM claw;
    @Override
    public void runOpMode() {
        claw = new ClawFSM();
        claw.init(hardwareMap);
        waitForStart();
        while(opModeIsActive()) {
            clawMove();
        }
    }

    public static double run = 1;

    void clawMove(){ //controls open and close with buttons
        if(gamepad1.right_bumper){
            claw.leftOpen();
        }
        else{
            claw.leftClose();
        }
        claw.update();
    }
}
