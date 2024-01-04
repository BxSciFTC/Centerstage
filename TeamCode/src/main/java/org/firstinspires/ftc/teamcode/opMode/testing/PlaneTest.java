//package org.firstinspires.ftc.teamcode.opMode.testing;
//
//import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
//import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
//
//import com.qualcomm.robotcore.hardware.HardwareMap;
//import org.firstinspires.ftc.teamcode.hardware.Plane;
//
//
//@TeleOp(name = "PlaneTest")
//public class PlaneTest extends LinearOpMode {
//    Plane plane = new Plane();
//    public void runOpMode(){
//        plane.init(hardwareMap);
//        waitForStart();
//        while(opModeIsActive()) {
//            planeFly();
//        }
//
//    }
//    void planeFly(){
//        if (gamepad1.left_bumper){
//            plane.planeState= Plane.PlaneState.ON;
//        }else {
//            plane.planeState= Plane.PlaneState.OFF;
//        }
//        plane.update();
//
//    }
//
//
//}
//
