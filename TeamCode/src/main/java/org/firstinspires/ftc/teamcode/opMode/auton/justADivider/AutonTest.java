package org.firstinspires.ftc.teamcode.opMode.auton.justADivider;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.hardware.Robot;
import org.firstinspires.ftc.vision.VisionPortal;

@Autonomous(name = "AutoTest")
public class AutonTest extends LinearOpMode {
    Robot robot = new Robot();
    VisionPortal myVisionPortal;
    RedPropThreshold colorProcessor = new RedPropThreshold();

    @Override
    public void runOpMode() throws InterruptedException {
        robot.init(hardwareMap);
        myVisionPortal = VisionPortal.easyCreateWithDefaults(hardwareMap.get(WebcamName.class, "webcam"), colorProcessor);

        waitForStart();
        while(opModeIsActive()) {
            telemetry.addData("pos:", colorProcessor.getPropPosition());


            telemetry.update();
        }
    }
}
