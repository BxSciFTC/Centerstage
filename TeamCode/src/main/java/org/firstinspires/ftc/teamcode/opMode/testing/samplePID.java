package org.firstinspires.ftc.teamcode.opMode.testing;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.control.PIDCoefficients;
import com.acmerobotics.roadrunner.control.PIDFController;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

@Disabled
@Config
@TeleOp(name = "hello")
public class samplePID extends LinearOpMode {

    DcMotor motor;
    public static double P = 0;
    public static double I = 0;
    public static double D = 0;
    public static double F = 0;
    PIDCoefficients coefficients = new PIDCoefficients(P, I, D);
    // create the controller
    PIDFController controller = new PIDFController(coefficients, 0, 0 ,0, (p, v) -> F);

    public static int wantToGoHere = 100;


    @Override
    public void runOpMode() throws InterruptedException {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        motor = hardwareMap.get(DcMotor.class, "test1");
        motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        motor.setTargetPosition(wantToGoHere);
        waitForStart();
        while (opModeIsActive()) {
            //motor is assumed to be paired with an encoder
            double power = controller.update(motor.getCurrentPosition());
            telemetry.addData("pos: ", motor.getCurrentPosition());
            telemetry.addData("target: ", wantToGoHere);
            telemetry.addData("power", power);
            motor.setPower(power);
        }
    }
}
