package org.firstinspires.ftc.teamcode.opMode.testing;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.teamcode.hardware.NewArm;
import org.firstinspires.ftc.teamcode.opMode.testing.armTesting.NewArm2;

@Disabled
@Config
@TeleOp(name="NewArmTest")
public class NewArmTest extends OpMode {
    NewArm2 arm = new NewArm2();
    public static int shoulderGo = 0;
    public static int elbowGo = 0;

    @Override
    public void init() {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        arm.init(hardwareMap);
    }

    @Override
    public void loop() {
        arm.shoulderGoToAngle(shoulderGo);
        arm.elbowGoToAngle(elbowGo);
        telemetry.addData("shoulderGo", shoulderGo);
        telemetry.addData("elbowGo", elbowGo);
        telemetry.addData("arm.shoulderDegrees()", arm.shoulderDegrees());
        telemetry.addData("arm.elbowDegrees()", arm.elbowDegrees());
        telemetry.addData("arm.shoulder.getCurrentPosition()", arm.shoulder.getCurrentPosition());
        telemetry.addData("arm.elbow.getCurrentPosition()", arm.elbow.getCurrentPosition());
        telemetry.addData("arm.getShoulderT()", arm.shoulderFF());
        telemetry.addData("arm.getElbowT()", arm.elbowFF());
    }
}
