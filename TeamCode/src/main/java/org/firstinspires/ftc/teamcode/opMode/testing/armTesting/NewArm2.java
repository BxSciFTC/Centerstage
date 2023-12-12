package org.firstinspires.ftc.teamcode.opMode.testing.armTesting;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.TouchSensor;
import org.firstinspires.ftc.teamcode.hardware.Mechanism;

@Config
@TeleOp(name = "NewArm2TestBasic")
public class NewArm2 extends LinearOpMode implements Mechanism {
    HardwareMap hwMap;
    DcMotorEx shoulder;
    DcMotorEx elbow;
    TouchSensor shoulderTouch;
    boolean prevShoulderTouch;
    TouchSensor elbowTouch;
    boolean prevElbowTouch;

    private PIDController shoulderController;
    public static double p1 = 0.03, i1 = 0, d1 = 0;

    private PIDController elbowController;
    public static double p2 = 0.03, i2 = 0, d2 = 0;

    public static int shoulderTarget = 0;
    public static int elbowTarget = 0;
    public int shoulderPos = 0;
    public int elbowPos = 0;

    public static double power1 = 0;
    public static double power2 = 0;


    @Override
    public void init(HardwareMap hwMap) {
        this.hwMap = hwMap;
        shoulderController = new PIDController(p1, i1, d1);
        elbowController = new PIDController(p2, i2, d2);

        shoulder = hwMap.get(DcMotorEx.class, "shoulder");
        elbow = hwMap.get(DcMotorEx.class, "elbow");

        shoulderTouch = hwMap.get(TouchSensor.class, "ShoulderTouch");
        elbowTouch = hwMap.get(TouchSensor.class, "ElbowTouch");

        prevShoulderTouch = shoulderTouch.isPressed();
        prevElbowTouch = elbowTouch.isPressed();
    }

    public void PIDUpdate() {
        shoulderController.setPID(p1, i1, d1);
        elbowController.setPID(p2, i2, d2);



        shoulderPos = shoulder.getCurrentPosition();
        double pid1  = shoulderController.calculate(shoulderPos, shoulderTarget);
        power1 = pid1 + shoulderFF();
        shoulder.setPower(power1);

        elbowPos = elbow.getCurrentPosition();
        double pid2  = elbowController.calculate(elbowPos, elbowTarget);
        power2 = pid2 + elbowFF();
        elbow.setPower(power2);

        if (shoulderTouch.isPressed()) {
            calibrateShoulder();
        } else {
            prevShoulderTouch = false;
        }
        if (elbowTouch.isPressed()) {
            calibrateElbow();
        } else {
            prevElbowTouch = false;
        }
    }

    public double shoulderFF() {
        return 0;
    }

    public double elbowFF() {
        return 0;
    }

    public void calibrateShoulder() {
        if (prevShoulderTouch == false) {
            shoulder.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            shoulder.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            prevShoulderTouch = true;
        } else
            return;
    }

    public void calibrateElbow() {
        if (prevElbowTouch == false) {
            elbow.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            elbow.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            prevElbowTouch = true;
        } else
            return;
    }

    @Override
    public void runOpMode() {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        init(hardwareMap);

        waitForStart();
        while (opModeIsActive()) {
            PIDUpdate();
            telemetry.addData("shoulderPos", shoulderPos);
            telemetry.addData("shoulderTarget", shoulderTarget);
            telemetry.addData("elbowPos", elbowPos);
            telemetry.addData("elbowTarget", elbowTarget);
            telemetry.addData("power1", power1);
            telemetry.addData("power2", power2);
            telemetry.update();
        }
    }
}
