package org.firstinspires.ftc.teamcode.opMode.auton.justADivider;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.opMode.auton.BlueFarTrapConstants;
import org.firstinspires.ftc.teamcode.opMode.auton.BlueNearTrapConstants;


import org.firstinspires.ftc.teamcode.opMode.auton.RedNearTrapConstants;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;

public class Park_RedNear extends LinearOpMode {
    SampleMecanumDrive drive;
    //ArmClawFSM deposit = new ArmClawFSM();
    @Override
    public void runOpMode() throws InterruptedException {
        drive = new SampleMecanumDrive(hardwareMap);
        //deposit.init(hardwareMap);


        TrajectorySequence move = drive.trajectorySequenceBuilder(RedNearTrapConstants.start)
                .strafeRight(48)
                .build();

        waitForStart();
        wait(10000);
        drive.followTrajectorySequenceAsync(move);

        while (opModeIsActive() && !isStopRequested()) {
            drive.update();
        }
    }
}
