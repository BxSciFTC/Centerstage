package org.firstinspires.ftc.teamcode.opMode.testing.armTesting;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Gamepad;

@Config
@TeleOp(name = "ArmMapperTest")
public class ArmMapperTest extends LinearOpMode {
    ArmMapper2 arm;


    private Gamepad gamepad11 = new Gamepad();

    private Gamepad gamepad22 = new Gamepad();

    public void runOpMode(){
        arm = new ArmMapper2();
        arm.init(hardwareMap);

        arm.moveTo(mapX, mapY);

        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        waitForStart();
        while(opModeIsActive()) {
            run(gamepad1);
        }
    }

    public static double mapX = 0;
    public static double mapY = 10;


    public void run(Gamepad gamepad) {
        gamepad22.copy(gamepad11);
        gamepad11.copy(gamepad);

        if (gamepad11.dpad_up && !gamepad22.dpad_up) {
            arm.shift(0, 2);
        }
        if (gamepad11.dpad_down && !gamepad22.dpad_down) {
            arm.shift(0, -2);
        }
        if (gamepad11.dpad_left && !gamepad22.dpad_left) {
            arm.shift(-2, 0);
        }
        if (gamepad11.dpad_right && !gamepad22.dpad_right) {
            arm.shift(2, 0);
        }

        arm.PIDUpdate();

        telemetry.addData("arm.shoulder.getCurrentPosition()", arm.arm.shoulder.getCurrentPosition());
        telemetry.addData("arm.elbow.getCurrentPosition()", arm.arm.elbow.getCurrentPosition());
        telemetry.addData("NewArm2.shoulderTarget", NewArm2.shoulderTarget);
        telemetry.addData("NewArm2.elbowTarget", NewArm2.elbowTarget);
        telemetry.addData("shoulder1", mapX);
        telemetry.addData("elbow1", mapY);
        telemetry.addData("arm.shoulderTouch.isPressed()", arm.arm.shoulderTouch.isPressed());
        telemetry.addData("arm.elbowTouch.isPressed()", arm.arm.elbowTouch.isPressed());
        telemetry.addData("arm.elbowTouch.isPressed()", arm.arm.elbowTouch.isPressed());

        double[] angles = arm.calculateAngle(mapX, mapY);

        telemetry.addData("angles[0]", angles[0]);
        telemetry.addData("angles[1]", angles[1]);
        telemetry.addData("angles[2]", angles[2]);
        telemetry.addData("arm.x", arm.xPos);
        telemetry.addData("arm.y", arm.yPos);
        telemetry.update();
    }
}
