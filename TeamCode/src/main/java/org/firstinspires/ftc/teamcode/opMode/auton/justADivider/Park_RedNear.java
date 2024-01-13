package org.firstinspires.ftc.teamcode.opMode.auton.justADivider;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.opMode.auton.BlueFarTrapConstants;
import org.firstinspires.ftc.teamcode.opMode.auton.BlueNearTrapConstants;


import org.firstinspires.ftc.teamcode.opMode.auton.RedNearTrapConstants;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;

public class Park_RedNear extends LinearOpMode {

    ElapsedTime timer;
    //ArmClawFSM deposit = new ArmClawFSM();
    @Override
    public void runOpMode() throws InterruptedException {

        DcMotor leftFront = hardwareMap.get(DcMotorEx.class, "leftFront");
        DcMotor leftRear = hardwareMap.get(DcMotorEx.class, "leftRear");
        DcMotor rightRear = hardwareMap.get(DcMotorEx.class, "rightRear");
        DcMotor rightFront = hardwareMap.get(DcMotorEx.class, "rightFront");
        leftFront.setDirection(DcMotor.Direction.REVERSE);
        rightFront.setDirection(DcMotor.Direction.REVERSE);
        leftRear.setDirection(DcMotor.Direction.REVERSE);
        rightRear.setDirection(DcMotor.Direction.REVERSE);
        //deposit.init(hardwareMap);


        waitForStart();
        wait(10000);


        while (opModeIsActive() && !isStopRequested()) {

            leftFront.setPower(0.5);
            leftRear.setPower(-0.5);
            rightFront.setPower((-0.5));
            rightRear.setPower((0.5));
            if(timer.milliseconds()>12000){
                leftFront.setPower(0);
                leftRear.setPower(-0);
                rightFront.setPower((-0));
                rightRear.setPower((0));
            }
        }
    }
}
