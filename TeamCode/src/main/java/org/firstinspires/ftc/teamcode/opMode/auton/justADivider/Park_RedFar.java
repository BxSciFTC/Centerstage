package org.firstinspires.ftc.teamcode.opMode.auton.justADivider;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.opMode.auton.BlueFarTrapConstants;
import org.firstinspires.ftc.teamcode.opMode.auton.BlueNearTrapConstants;


import org.firstinspires.ftc.teamcode.opMode.auton.RedFarTrapConstants;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;

public class Park_RedFar extends LinearOpMode {
    SampleMecanumDrive drive;
    //ArmClawFSM deposit = new ArmClawFSM();
    @Override
    public void runOpMode() throws InterruptedException {
        drive = new SampleMecanumDrive(hardwareMap);
        DcMotor leftFront = hardwareMap.get(DcMotorEx.class, "leftFront");
        DcMotor leftRear = hardwareMap.get(DcMotorEx.class, "leftRear");
        DcMotor rightRear = hardwareMap.get(DcMotorEx.class, "rightRear");
        DcMotor rightFront = hardwareMap.get(DcMotorEx.class, "rightFront");
        //deposit.init(hardwareMap);


        waitForStart();
        wait(10000);


        while (opModeIsActive() && !isStopRequested()) {
            drive.update();
            leftFront.setPower(0.5);
            leftRear.setPower(-0.5);
            rightFront.setPower((-0.5));
            rightRear.setPower((0.5));
        }
    }
}
