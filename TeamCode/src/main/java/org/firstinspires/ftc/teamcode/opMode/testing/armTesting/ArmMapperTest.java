package org.firstinspires.ftc.teamcode.opMode.testing.armTesting;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

@Config
@TeleOp(name = "ArmMapperTest")
public class ArmMapperTest extends LinearOpMode {
    ArmMapper2 arm;


    private Gamepad gamepad1 = new Gamepad();

    private Gamepad gamepad2 = new Gamepad();

    public void runOpMode(){
        arm = new ArmMapper2();
        arm.init(hardwareMap);

//        arm.shift(-5, 0);

        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        waitForStart();
        while(opModeIsActive()) {
            run(gamepad1);
        }
    }

    public static double shoulder1 = 14.14;
    public static double elbow1 = 14.14;




    public void run(Gamepad gamepad) {
        gamepad2.copy(gamepad1);
        gamepad1.copy(gamepad);
//        arm.moveTo(shoulder1, elbow1);


        if (gamepad1.cross) {
            arm.arm.shoulder.setPower(1);
            arm.shift(0.1, 0);
        }
        if (gamepad.triangle) {
            arm.arm.shoulder.setPower(1);
            arm.shift(0, -0.1);
        }

//        arm.PIDUpdate();

        telemetry.addData("arm.shoulder.getCurrentPosition()", arm.arm.shoulder.getCurrentPosition());
        telemetry.addData("arm.elbow.getCurrentPosition()", arm.arm.elbow.getCurrentPosition());
        telemetry.addData("NewArm2.shoulderTarget", NewArm2.shoulderTarget);
        telemetry.addData("NewArm2.elbowTarget", NewArm2.elbowTarget);
        telemetry.addData("shoulder1", shoulder1);
        telemetry.addData("elbow1", elbow1);
        telemetry.addData("arm.shoulderTouch.isPressed()", arm.arm.shoulderTouch.isPressed());
        telemetry.addData("arm.elbowTouch.isPressed()", arm.arm.elbowTouch.isPressed());
        telemetry.addData("arm.elbowTouch.isPressed()", arm.arm.elbowTouch.isPressed());

        double[] angles = arm.calculateAngle(shoulder1, elbow1);

        telemetry.addData("angles[0]", angles[0]);
        telemetry.addData("angles[1]", angles[1]);
        telemetry.addData("angles[2]", angles[2]);
        telemetry.addData("arm.x", arm.x);
        telemetry.addData("arm.y", arm.y);
        telemetry.update();
    }
}
