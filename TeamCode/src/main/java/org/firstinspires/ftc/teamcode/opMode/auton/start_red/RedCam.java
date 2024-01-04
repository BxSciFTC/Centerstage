//package org.firstinspires.ftc.teamcode.opMode.auton.start_red;
//import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
//
//import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
//import org.firstinspires.ftc.teamcode.hardware.ArmClawFSM;
//import org.firstinspires.ftc.teamcode.hardware.Camera;
//import org.firstinspires.ftc.teamcode.opMode.auton.Constants;
//import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;
//
////need to update with constants
//public class RedCam extends LinearOpMode {
//    SampleMecanumDrive drive;
//    Camera camera;
//    ArmClawFSM score;
//    private int region;
//
//    @Override
//    public void runOpMode() throws InterruptedException {
//        drive = new SampleMecanumDrive(hardwareMap);
//        camera = new Camera();
//
//
//        camera.init(hardwareMap);
//        score.init(hardwareMap);
//
//        drive.setPoseEstimate(Constants.BACKSTAGE_BLUE_START);
//
//        TrajectorySequence left = drive.trajectorySequenceBuilder(Constants.)
//                .splineTo(Constants.LEFT_SPIKE_MARK, 0)
//                .build();
//
//        TrajectorySequence middle = drive.trajectorySequenceBuilder(Constants.)
//                .forward(28)
//                .build();
//
//        TrajectorySequence right = drive.trajectorySequenceBuilder(Constants.)
//                .splineTo(Constants.RIGHT_SPIKE_MARK, 0)
//                .build();
//
//        while (opModeIsActive() && isStopRequested()) {
//            region = camera.whichRegion();
//            telemetry.addData("Region: ", region);
//            telemetry.update();
//        }
//        waitForStart();
//        camera.stopStreaming();
//        if (region == 1) {
//        drive.followTrajectorySequenceAsync(left);
//
//        } else if (region == 3) {
//            drive.followTrajectorySequenceAsync(right);
//
//        } else {
//            drive.followTrajectorySequenceAsync(middle);
//        }
//
//
//        while (opModeIsActive() && !isStopRequested()) {
//            drive.update();
//
//            telemetry.addData("Region: ", region);
//            telemetry.update();
//        }
//    }
//}
