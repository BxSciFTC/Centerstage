package org.firstinspires.ftc.teamcode.opMode.testing.ArmPresetsTesting;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Gamepad;
import org.firstinspires.ftc.teamcode.opMode.testing.armTesting.NewArm2;

@Config
@TeleOp(name = "ArmPresetsTest")
public class ArmPresetsTest extends LinearOpMode {
    ArmPresetsMotion arm;

    Gamepad prev = new Gamepad();
    String mode = "";

    public void runOpMode() {
        arm = new ArmPresetsMotion();
        arm.init(hardwareMap);
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        prev.copy(gamepad1);

        waitForStart();
        while (opModeIsActive()) {
            if (gamepad1.dpad_up && !prev.dpad_up) {
                mode = "Rest";
                arm.rest();
            }
            if (gamepad1.dpad_down && !prev.dpad_down) {
                mode = "pickup";
                arm.pickup();
            }
            if (gamepad1.dpad_left && !prev.dpad_left) {
                mode = "Score1";
                arm.score1();
            }
            if (gamepad1.dpad_right && !prev.dpad_right) {
                mode = "score2";
                arm.score2();
            }

            telemetry.addData("shoulder pos", arm.arm.shoulder.getCurrentPosition());
            telemetry.addData("elbow pos", arm.arm.elbow.getCurrentPosition());
            telemetry.addData("shoulderTarget", NewArm2.shoulderTarget);
            telemetry.addData("elbowTarget", NewArm2.elbowTarget);
            telemetry.addData("arm.arm.elbowDegrees()", arm.arm.elbowDegrees());
            telemetry.addData("arm.arm.shoulderDegrees()", arm.arm.shoulderDegrees());
            telemetry.addData("mode", mode);
//            telemetry.addData("presets", arm.presets);
            telemetry.update();
            prev.copy(gamepad1);
        }

    }
}