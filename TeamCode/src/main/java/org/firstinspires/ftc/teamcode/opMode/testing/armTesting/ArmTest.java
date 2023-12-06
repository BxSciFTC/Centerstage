package org.firstinspires.ftc.teamcode.opMode.testing.armTesting;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

@TeleOp(name = "ArmTest")
public class ArmTest extends LinearOpMode {
    NewArm arm;

    private Gamepad gamepad1 = new Gamepad();

    private Gamepad gamepad2 = new Gamepad();

    public void runOpMode(){
        arm = new NewArm();
        arm.init(hardwareMap);

        telemetry.addData("TeleOpMain", "boop1");
        telemetry.update();
        waitForStart();
        while(opModeIsActive()) {
            run(gamepad1);
        }
        telemetry.addData("TeleOpMain", "boop2");
        telemetry.update();


    }
    public void run(Gamepad gamepad) {
        gamepad2.copy(gamepad1);
        gamepad1.copy(gamepad);
        arm.PIDUpdate();
    }
}
