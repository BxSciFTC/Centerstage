package org.firstinspires.ftc.teamcode.opMode.testing.armTesting;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

@Config
@TeleOp(name = "ArmTest")
public class ArmTest extends LinearOpMode {
    NewArm2 arm;

    private Gamepad gamepad1 = new Gamepad();

    private Gamepad gamepad2 = new Gamepad();

    public void runOpMode(){
        arm = new NewArm2();
        arm.init(hardwareMap);

        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        waitForStart();
        while(opModeIsActive()) {
            run(gamepad1);

        }


    }
    
    public static int shoulder1 = 0;
    public static int elbow1 = 27;
    
    
    public void run(Gamepad gamepad) {
        gamepad2.copy(gamepad1);
        gamepad1.copy(gamepad);

        arm.shoulderGoToAngle(shoulder1);
        arm.elbowGoToAngle(elbow1);
        arm.PIDUpdate();
        
        telemetry.addData("arm.shoulder.getCurrentPosition()", arm.shoulder.getCurrentPosition());
        telemetry.addData("arm.elbow.getCurrentPosition()", arm.elbow.getCurrentPosition());
        telemetry.addData("NewArm2.shoulderTarget", NewArm2.shoulderTarget);
        telemetry.addData("NewArm2.elbowTarget", NewArm2.elbowTarget);
        telemetry.addData("shoulder1", shoulder1);
        telemetry.addData("elbow1", elbow1);
        telemetry.addData("arm.shoulderTouch.isPressed()", arm.shoulderTouch.isPressed());
        telemetry.addData("arm.elbowTouch.isPressed()", arm.elbowTouch.isPressed());
        telemetry.addData("arm.shoulderDegrees()", arm.shoulderDegrees());
        telemetry.addData("arm.elbowDegrees()", arm.elbowDegrees());
        telemetry.addData("arm.COMAngle", arm.COMAngle);

        telemetry.update();
    }
}
