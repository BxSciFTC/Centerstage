package org.firstinspires.ftc.teamcode.opMode.auton.autons;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.teamcode.hardware.ClawFSM;
import org.firstinspires.ftc.teamcode.hardware.MecanumDrive;

@Autonomous(name = "leftPark")
public class LeftPark extends LinearOpMode {
    MecanumDrive drive;
    ElapsedTime time;
    ClawFSM claw;


    @Override
    public void runOpMode() throws InterruptedException {
        drive = new MecanumDrive();
        drive.init(hardwareMap);
        claw = new ClawFSM();
        claw.init(hardwareMap);
        claw.setPreAutonRest();
        time = new ElapsedTime();

        waitForStart();
        time.reset();
        while (opModeIsActive() && !isStopRequested()) {
            if (time.milliseconds() < 2000) {
                drive.setWeightedDrivePowerByConstants(-0.5, 0, 0);
            } else {
                drive.setWeightedDrivePowerByConstants(0, 0, 0);
                claw.setRest();
                claw.leftOpen();
                claw.rightOpen();
                claw.update();
            }
        }
    }
}
