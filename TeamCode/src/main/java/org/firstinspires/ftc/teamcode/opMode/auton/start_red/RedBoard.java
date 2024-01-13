package org.firstinspires.ftc.teamcode.opMode.auton.start_red;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.hardware.ArmClawFSM;
import org.firstinspires.ftc.teamcode.opMode.auton.Constants;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.opMode.auton.RedNearCornerConstants;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;
@Autonomous(name = "Red Close", group = "_Auto")
//need to update with constants
public class RedBoard extends LinearOpMode {
    SampleMecanumDrive drive;
    //ArmClawFSM deposit = new ArmClawFSM();

    @Override
    public void runOpMode() throws InterruptedException {
        drive = new SampleMecanumDrive(hardwareMap);
        //deposit.init(hardwareMap);

        TrajectorySequence lets_go = drive.trajectorySequenceBuilder(RedNearCornerConstants.start)
                .forward(28)
                .back(2)
                .strafeRight(60)
                .build();

        waitForStart();

        drive.followTrajectorySequenceAsync(lets_go);

        while (opModeIsActive() && !isStopRequested()) {
            drive.update();
        }
    }
}
//