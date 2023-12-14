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
    ArmMapper2 arm;
    

    private Gamepad gamepad1 = new Gamepad();

    private Gamepad gamepad2 = new Gamepad();

    public void runOpMode(){
        arm = new ArmMapper2();
        arm.init(hardwareMap);

        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        waitForStart();
        while(opModeIsActive()) {
            run(gamepad1);

        }


    }
    
    public static int shoulder1 = 0;
    public static int elbow1 = 0;
    
    
    public void run(Gamepad gamepad) {
        gamepad2.copy(gamepad1);
        gamepad1.copy(gamepad);
        arm.moveTo(shoulder1, elbow1);
//        arm.shoulder.setTargetPosition(shoulder1);
//        arm.elbow.setTargetPosition(elbow1);
        arm.PIDUpdate();
        
        telemetry.addData("arm.shoulder.getCurrentPosition()", arm.arm.shoulder.getCurrentPosition());
        telemetry.addData("arm.elbow.getCurrentPosition()", arm.arm.elbow.getCurrentPosition());
        telemetry.addData("shoulder1", shoulder1);
        telemetry.addData("elbow1", elbow1);
        telemetry.addData("arm.shoulderTouch.isPressed()", arm.arm.shoulderTouch.isPressed());
        telemetry.addData("arm.elbowTouch.isPressed()", arm.arm.elbowTouch.isPressed());
//        telemetry.addData("", arm.);
//        telemetry.addData("", );
//        telemetry.addData("", );
//        telemetry.addData("", );
        telemetry.update();
    }
}
