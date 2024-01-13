package org.firstinspires.ftc.teamcode.opMode.auton.justADivider;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import org.firstinspires.ftc.teamcode.R;
import org.firstinspires.ftc.teamcode.hardware.Camera;
import org.firstinspires.ftc.teamcode.hardware.Robot;

@Autonomous(name = "cameraTest")
public class CameraTest extends LinearOpMode {
    Camera camera;
    int region = 1;
//    Robot robot;

//    DcMotor leftFront = hardwareMap.get(DcMotorEx .class, "leftFront");
//    DcMotor leftRear = hardwareMap.get(DcMotorEx.class, "leftRear");
//    DcMotor rightFront = hardwareMap.get(DcMotorEx.class, "rightRear");
//    DcMotor rightRear = hardwareMap.get(DcMotorEx.class, "rightFront");
//
//    leftFront.setPower((y + x + rx) /denominator);
//    leftRear.setPower((y - x + rx)  /denominator);
//    rightFront.setPower((y - x - rx)/denominator);
//    rightRear.setPower((y + x - rx) /denominator);
//    USE AN ELAPSEDTIME AND DO SOME RUN FOR x SECONDS SHENANIGANS

    @Override
    public void runOpMode()  {
//        robot = new Robot();
//        robot.init(hardwareMap);


        camera = new Camera("blue");

        camera.init(hardwareMap);



        waitForStart();

        while (opModeIsActive() && !isStopRequested()) {
            region = camera.whichRegion();
            telemetry.addData("Region: ", region);
            telemetry.update();
        }


        while (opModeIsActive() && isStopRequested()) {
            region = camera.whichRegion();
            telemetry.addData("Region: ", region);
            telemetry.update();
        }
        camera.stopStreaming();

        if (region == 1) {
            //left
        } else if (region == 3) {
            //right
        } else {
            //middle
        }
    }
}
