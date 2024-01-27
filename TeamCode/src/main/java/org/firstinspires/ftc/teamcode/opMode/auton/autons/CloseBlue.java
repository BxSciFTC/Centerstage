package org.firstinspires.ftc.teamcode.opMode.auton.autons;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.hardware.Robot;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;


//NOT DONE
@Autonomous(name = "CloseBlue")
public class CloseBlue extends LinearOpMode {
    Robot robot = new Robot();
    SampleMecanumDrive drive;

    @Override
    public void runOpMode() {
        //init
        drive = new SampleMecanumDrive(hardwareMap);
        robot.init(hardwareMap);

        drive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        drive.setPoseEstimate(new Pose2d(-35, -61, Math.toRadians(90)));
        TrajectorySequence initial = drive.trajectorySequenceBuilder(new Pose2d(-35, -61, Math.toRadians(90)))
                .lineToSplineHeading(new Pose2d(-35, -10, Math.toRadians(270)))
                .UNSTABLE_addTemporalMarkerOffset(0, () -> {

                })
                .waitSeconds(1)
                .lineToLinearHeading(new Pose2d(34, -10, Math.toRadians(0)))
                .splineToSplineHeading(new Pose2d(39, -34, Math.toRadians(0)), Math.toRadians(270))
                .UNSTABLE_addTemporalMarkerOffset(0, () -> {

                })
                .waitSeconds(1)
                .splineToLinearHeading(new Pose2d(34, -10, Math.toRadians(180)), Math.toRadians(0))
                .splineToLinearHeading(new Pose2d(-42, -10, Math.toRadians(180)), Math.toRadians(0))
                .UNSTABLE_addTemporalMarkerOffset(0, () -> {

                })
                .lineToLinearHeading(new Pose2d(34, -10, Math.toRadians(0)))
                .splineToSplineHeading(new Pose2d(39, -34, Math.toRadians(0)), Math.toRadians(270))
                .UNSTABLE_addTemporalMarkerOffset(0, () -> {

                })
                .waitSeconds(1)
                .lineToLinearHeading(new Pose2d(50, -57, Math.toRadians(180)))
                .build();

        waitForStart();
        drive.followTrajectorySequence(initial);
        while (opModeIsActive() && !isStopRequested()) {
            drive.update();
        }
    }
}
