package org.firstinspires.ftc.teamcode.opMode.auton.start_blue;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.opMode.auton.constsThings.BlueNearCornerConstants;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;

@Autonomous(name = "BlueBoard", group = "_Auto")
//need to update with constants
public class BlueBoard extends LinearOpMode {
    SampleMecanumDrive drive;
    //ArmClawFSM deposit = new ArmClawFSM();
    @Override
    public void runOpMode() throws InterruptedException {
        drive = new SampleMecanumDrive(hardwareMap);
        //deposit.init(hardwareMap);

        TrajectorySequence move = drive.trajectorySequenceBuilder(BlueNearCornerConstants.start)
                .forward(28)
                .back(2)
                .strafeLeft(60)
                .build();

        waitForStart();

        drive.followTrajectorySequenceAsync(move);

        while (opModeIsActive() && !isStopRequested()) {
            drive.update();
        }
    }
}
