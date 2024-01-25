package org.firstinspires.ftc.teamcode.opMode.auton.autons;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.hardware.Robot;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;

@Autonomous(name = "FarRed")
public class FarRed extends LinearOpMode {
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
                .lineToConstantHeading(new Vector2d(40, 40))
                .build()
                ;



        telemetry.setMsTransmissionInterval(50);
        /*
         * The INIT-loop:
         * This REPLACES waitForStart!
         */
        while (!isStarted() && !isStopRequested())
        {


            telemetry.update();
            sleep(20);
        }
    }

}
