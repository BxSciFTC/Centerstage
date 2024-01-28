package org.firstinspires.ftc.teamcode.opMode.auton.autons;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.teamcode.hardware.MecanumDrive;

@Autonomous(name = "rightPark")
public class RightPark extends LinearOpMode {
    MecanumDrive drive;
    ElapsedTime time;


    @Override
    public void runOpMode() throws InterruptedException {
        drive = new MecanumDrive();
        drive.init(hardwareMap);
        time = new ElapsedTime();

        waitForStart();
        time.reset();
        while (opModeIsActive() && !isStopRequested()) {
            if (time.milliseconds() < 2000) {
                drive.setWeightedDrivePowerByConstants(0.5, 0, 0);
            } else {
                drive.setWeightedDrivePowerByConstants(0, 0, 0);
            }
        }
    }
}