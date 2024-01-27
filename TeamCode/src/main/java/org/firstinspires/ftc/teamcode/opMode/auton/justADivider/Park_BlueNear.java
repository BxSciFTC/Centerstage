package org.firstinspires.ftc.teamcode.opMode.auton.justADivider;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.ElapsedTime;

@Autonomous(name = "Park_blueN")
public class Park_BlueNear extends LinearOpMode {
    private DcMotorEx leftFront;
    private DcMotorEx leftRear;
    private DcMotorEx rightFront;
    private DcMotorEx rightRear;
    ElapsedTime timer;
    //ArmClawFSM deposit = new ArmClawFSM();
    @Override
    public void runOpMode() throws InterruptedException {

        leftFront = hardwareMap.get(DcMotorEx.class, "leftFront");
       leftRear = hardwareMap.get(DcMotorEx.class, "leftRear");
         rightRear = hardwareMap.get(DcMotorEx.class, "rightRear");
         rightFront = hardwareMap.get(DcMotorEx.class, "rightFront");
        leftRear.setDirection(DcMotor.Direction.REVERSE);
        waitForStart();

        while(timer.milliseconds()<4000){

        }
        leftFront.setPower(0.5);
        leftRear.setPower(-0.5);
        rightFront.setPower(-0.5);
        rightRear.setPower(0.5);
        while(timer.milliseconds()<6000) {

        }
        leftFront.setPower(0);
        leftRear.setPower(0);
        rightFront.setPower(0);
        rightRear.setPower(0);
        while (opModeIsActive() && !isStopRequested()) {

        }
    }
}
