//package org.firstinspires.ftc.teamcode.opMode.auton.start_red;
//import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
//import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
//
//import org.firstinspires.ftc.teamcode.hardware.ArmClawFSM;
//import org.firstinspires.ftc.teamcode.opMode.auton.Constants;
//import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
//import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;
//
////need to update with constants
//@Autonomous(name = "Red Far", group = "_Auto")
//public class RedFar extends LinearOpMode {
//    ArmClawFSM deposit = new ArmClawFSM();
//    SampleMecanumDrive drive;
//    @Override
//    public void runOpMode() throws InterruptedException {
//        drive = new SampleMecanumDrive(hardwareMap);
//        deposit.init(hardwareMap);
//
//        TrajectorySequence move = drive.trajectorySequenceBuilder(Constants.//need to add update with constants)
//                .forward(28)
//                .back(2)
//                .strafeLeft(84)
//                .build();
//
//        waitForStart();
//
//        drive.followTrajectorySequenceAsync(move);
//
//        while (opModeIsActive() && !isStopRequested()) {
//            drive.update();
//        }
//    }
//}