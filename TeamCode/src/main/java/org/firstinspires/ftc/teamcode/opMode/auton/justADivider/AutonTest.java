package org.firstinspires.ftc.teamcode.opMode.auton.justADivider;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.hardware.Robot;
import org.firstinspires.ftc.vision.VisionPortal;

//https://deltacv.gitbook.io/eocv-sim/vision-portal/introduction-to-visionportal/using-visionportal-within-opmodes

//https://ftc-code.gitbook.io/tech-toolbox/computer-vision

//https://ftc-docs.firstinspires.org/en/latest/apriltag/vision_portal/visionportal_overview/visionportal-overview.html


@Autonomous(name = "AutoTest")
public class AutonTest extends LinearOpMode {
    Robot robot = new Robot();
    VisionPortal myVisionPortal;
    RedPropThreshold colorProcessor = new RedPropThreshold();

    @Override
    public void runOpMode() throws InterruptedException {
        robot.init(hardwareMap);
        myVisionPortal = VisionPortal.easyCreateWithDefaults(hardwareMap.get(WebcamName.class, "Webcam 1"), colorProcessor);

        myVisionPortal.resumeStreaming();

        waitForStart();
        while(opModeIsActive()) {
            telemetry.addData("pos:", colorProcessor.getPropPosition());


            telemetry.update();
        }
    }
}
