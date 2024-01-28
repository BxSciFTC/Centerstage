package org.firstinspires.ftc.teamcode.opMode.auton.justADivider;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.teamcode.hardware.Camera;
import org.firstinspires.ftc.teamcode.hardware.Robot;

@Disabled
@Autonomous(name = "blue")
public class Blue extends LinearOpMode {
    Robot robot;
    Camera camera;
    ElapsedTime time;
    int region = 1;
//    Robot robot;

    DcMotor leftFront = hardwareMap.get(DcMotorEx.class, "leftFront");
    DcMotor leftRear = hardwareMap.get(DcMotorEx.class, "leftRear");
    DcMotor rightRear = hardwareMap.get(DcMotorEx.class, "rightRear");
    DcMotor rightFront = hardwareMap.get(DcMotorEx.class, "rightFront");

//    USE AN ELAPSEDTIME AND DO SOME RUN FOR x SECONDS SHENANIGANS

    @Override
    public void runOpMode()  {
        robot = new Robot();
        robot.init(hardwareMap);


        camera = new Camera("blue");

        camera.init(hardwareMap);

        time = new ElapsedTime();

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

        time.reset();

        leftFront.setPower((0.5) );
        leftRear.setPower((0.5)  );
        rightFront.setPower((0.5));
        rightRear.setPower((0.5) );

//        robot.arm.placeInBasket();
        while (time.milliseconds() < 3000) {};

        if (region == 1) {
            //left
            leftFront.setPower((-0.5) );
            leftRear.setPower((-0.5)  );
            rightFront.setPower((0.5));
            rightRear.setPower((0.5) );
            while (time.milliseconds() < 1500) {};
        } else if (region == 3) {
            //right
            leftFront.setPower((0.5) );
            leftRear.setPower((0.5)  );
            rightFront.setPower((-0.5));
            rightRear.setPower((-0.5) );
            while (time.milliseconds() < 1500) {};
        } else {
            //middle
        }
        time.reset();

        robot.claw.leftOpen();
    }
}
