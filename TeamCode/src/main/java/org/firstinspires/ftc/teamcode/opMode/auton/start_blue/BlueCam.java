//package org.firstinspires.ftc.teamcode.opMode.auton.start_blue;
//import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
//
//import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
//import org.firstinspires.ftc.teamcode.hardware.ArmClawFSM;
//import org.firstinspires.ftc.teamcode.hardware.Camera;
//import org.firstinspires.ftc.teamcode.opMode.auton.constsThings.BlueFarCornerConstants;
//import org.firstinspires.ftc.teamcode.opMode.auton.constsThings.Constants;
//import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;
//
//public class BlueCam extends LinearOpMode {
//    SampleMecanumDrive drive;
//    Camera camera;
//    ArmClawFSM score;
//    private int region;
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
//        TrajectorySequence left = drive.trajectorySequenceBuilder(BlueFarCornerConstants.start)
//                //.splineTo( ) Need to update
//                .build();
//
//        TrajectorySequence middle = drive.trajectorySequenceBuilder(BlueFarCornerConstants.start)
//                .forward(28)
//                .build();
//
//        TrajectorySequence right = drive.trajectorySequenceBuilder(BlueFarCornerConstants.start)
//                //.splineTo(, 0) Need to update
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
//            drive.followTrajectorySequenceAsync(left);
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
