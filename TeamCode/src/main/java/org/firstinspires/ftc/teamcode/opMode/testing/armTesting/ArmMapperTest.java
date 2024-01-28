package org.firstinspires.ftc.teamcode.opMode.testing.armTesting;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Gamepad;
import org.firstinspires.ftc.teamcode.hardware.Plane;

@Disabled
@Config
@TeleOp(name = "ArmMapperTest")
public class ArmMapperTest extends LinearOpMode {
    ArmMapper2 arm;
    Plane plane;


    private Gamepad gamepad11 = new Gamepad();

    private Gamepad gamepad22 = new Gamepad();

    public void runOpMode(){
        arm = new ArmMapper2();
        arm.init(hardwareMap);
        plane = new Plane();
        plane.init(hardwareMap);
//        arm.moveToMotionProfile(mapX, mapY);

        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        waitForStart();
        while(opModeIsActive()) {
            run(gamepad1);
        }

    }

    public static double mapX = 0;
    public static double mapY = 24;
    public static double q1 = 90;
    public static double q2 = 180;


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
        if (gamepad1.x) {
            arm.moveToMotionProfileByAngle(q1, q2, 10);
            telemetry.addData("PRESSED", "AKJDASKDKJAWDKJA");
            plane.fire();
        }

        arm.PIDUpdate();

        telemetry.addData("arm.shoulder.getCurrentPosition()", arm.arm.shoulder.getCurrentPosition());
        telemetry.addData("arm.elbow.getCurrentPosition()", arm.arm.elbow.getCurrentPosition());
        telemetry.addData("NewArm2.shoulderTarget", NewArm2.shoulderTarget);
        telemetry.addData("NewArm2.elbowTarget", NewArm2.elbowTarget);
        telemetry.addData("shoulder1", mapX);
        telemetry.addData("elbow1", mapY);
        telemetry.addData("busy", arm.busy);
        telemetry.addData("arm.arm.elbowDegrees()", arm.arm.elbowDegrees());
        telemetry.addData("arm.arm.shoulderDegrees()", arm.arm.shoulderDegrees());
        telemetry.addData("arm.xPos", arm.xPos);
        telemetry.addData("arm.yPos", arm.yPos);
        telemetry.addData("arm.shoulderTouch.isPressed()", arm.arm.shoulderTouch.isPressed());
        telemetry.addData("ArmMapper2.shoulderDx", ArmMapper2.shoulderDx);
        telemetry.addData("ArmMapper2.elbowDx", ArmMapper2.elbowDx);

//        double[] angles = arm.calculateAngle(arm.xPos, arm.yPos);
//
//        telemetry.addData("angles[0]", angles[0]);
//        telemetry.addData("angles[1]", angles[1]);
//        telemetry.addData("angles[2]", angles[2]);
//        telemetry.addData("arm.x", arm.xPos);
//        telemetry.addData("arm.y", arm.yPos);
        telemetry.update();
    }
}
