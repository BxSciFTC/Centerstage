package org.firstinspires.ftc.teamcode.opMode.testing.armTesting;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Gamepad;
import org.firstinspires.ftc.teamcode.hardware.Plane;

@Config
@TeleOp(name = "ArmMapperTest2")
public class ArmMapperTest2 extends LinearOpMode {
    ArmMapper3 arm;
    Plane plane;


    private Gamepad gamepad11 = new Gamepad();

    private Gamepad gamepad22 = new Gamepad();

    public void runOpMode(){
        arm = new ArmMapper3();
        arm.init(hardwareMap);
        plane = new Plane();
        plane.init(hardwareMap);

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

        if (gamepad1.cross) {
            arm.moveToMotionProfileByAngle(q1, q2);
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
        telemetry.addData("arm.shoulderTouch.isPressed()", arm.arm.shoulderTouch.isPressed());
        telemetry.addData("ArmMapper2.shoulderDx", ArmMapper2.shoulderDx);
        telemetry.addData("ArmMapper2.elbowDx", ArmMapper2.elbowDx);

        telemetry.update();
    }
}
