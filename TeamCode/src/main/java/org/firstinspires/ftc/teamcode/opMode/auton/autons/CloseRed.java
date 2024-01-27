package org.firstinspires.ftc.teamcode.opMode.auton.autons;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.hardware.Camera;
import org.firstinspires.ftc.teamcode.hardware.Robot;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;

//TEST1
@Autonomous(name = "CloseRed")
public class CloseRed extends LinearOpMode {
    Robot robot = new Robot();
    SampleMecanumDrive drive;
    Camera camera;
    int region = 1;

    @Override
    public void runOpMode() {
        //init
        drive = new SampleMecanumDrive(hardwareMap);
        robot.init(hardwareMap);
        camera = new Camera("blue");
        camera.init(hardwareMap);

        drive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        drive.setPoseEstimate(new Pose2d(11, -61, Math.toRadians(90)));

        TrajectorySequence leftSpikeMark = drive.trajectorySequenceBuilder(new Pose2d(11, -61, Math.toRadians(90)))
                .lineToSplineHeading(new Pose2d(11, -52, Math.toRadians(110)))
                .UNSTABLE_addTemporalMarkerOffset(0, () -> {

                })
                .waitSeconds(1)
                .build();

        TrajectorySequence middleSpikeMark = drive.trajectorySequenceBuilder(new Pose2d(11, -61, Math.toRadians(90)))
                .lineToSplineHeading(new Pose2d(11, -52, Math.toRadians(90)))
                .UNSTABLE_addTemporalMarkerOffset(0, () -> {

                })
                .waitSeconds(1)
                .build();

        TrajectorySequence rightSpikeMark = drive.trajectorySequenceBuilder(new Pose2d(11, -61, Math.toRadians(90)))
                .lineToSplineHeading(new Pose2d(11, -52, Math.toRadians(70)))
                .UNSTABLE_addTemporalMarkerOffset(0, () -> {

                })
                .waitSeconds(1)
                .build();



        TrajectorySequence scoreLeft = drive.trajectorySequenceBuilder(leftSpikeMark.end())
                .lineToLinearHeading(new Pose2d(39, -48, Math.toRadians(0)))
                .setTangent(90)
                .splineToSplineHeading(new Pose2d(39, -34, Math.toRadians(0)), Math.toRadians(0))
                .UNSTABLE_addTemporalMarkerOffset(0, () -> {

                })
                .waitSeconds(1)
                .lineToLinearHeading(new Pose2d(50, -57, Math.toRadians(180)))
                .build();

        TrajectorySequence scoreMiddle = drive.trajectorySequenceBuilder(leftSpikeMark.end())
                .lineToLinearHeading(new Pose2d(39, -48, Math.toRadians(0)))
                .setTangent(90)
                .splineToSplineHeading(new Pose2d(39, -34, Math.toRadians(0)), Math.toRadians(0))
                .UNSTABLE_addTemporalMarkerOffset(0, () -> {

                })
                .waitSeconds(1)
                .lineToLinearHeading(new Pose2d(50, -57, Math.toRadians(180)))
                .build();

        TrajectorySequence scoreRight = drive.trajectorySequenceBuilder(leftSpikeMark.end())
                .lineToLinearHeading(new Pose2d(39, -48, Math.toRadians(0)))
                .setTangent(90)
                .splineToSplineHeading(new Pose2d(39, -34, Math.toRadians(0)), Math.toRadians(0))
                .UNSTABLE_addTemporalMarkerOffset(0, () -> {

                })
                .waitSeconds(1)
                .lineToLinearHeading(new Pose2d(50, -57, Math.toRadians(180)))
                .build();


        while (!isStarted() && !isStopRequested()) {
            region = camera.whichRegion();
            telemetry.addData("Region", camera.whichRegion());
            telemetry.update();
        }
        camera.stopStreaming();

        Pose2d end;

        if (region == 1) {
            drive.followTrajectorySequenceAsync(rightSpikeMark);
            drive.followTrajectorySequence(scoreRight);
        } else if (region == 2) {
            drive.followTrajectorySequenceAsync(middleSpikeMark);
            drive.followTrajectorySequence(scoreMiddle);
        } else {
            drive.followTrajectorySequenceAsync(leftSpikeMark);
            drive.followTrajectorySequence(scoreLeft);
        }

        while (opModeIsActive() && !isStopRequested()) {
            drive.update();
        }
    }
}
