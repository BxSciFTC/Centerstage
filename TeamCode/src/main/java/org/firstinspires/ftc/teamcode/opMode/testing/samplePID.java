package org.firstinspires.ftc.teamcode.opMode.testing;

import com.acmerobotics.roadrunner.control.PIDCoefficients;
import com.acmerobotics.roadrunner.control.PIDFController;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOp(name = "hello")
public class samplePID extends LinearOpMode {

    DcMotor motor;
    double P = 0;
    double I = 0;
    double D = 0;
    double F = 0;
    PIDCoefficients coefficients = new PIDCoefficients(P, I, D);
    // create the controller
    PIDFController controller = new PIDFController(coefficients, 0, 0 ,0, (p, v) -> F);

    int wantToGoHere = 100;

    @Override
    public void runOpMode() throws InterruptedException {
        motor = hardwareMap.get(DcMotor.class, "motor");
        motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        motor.setTargetPosition(wantToGoHere);
        waitForStart();
        while (opModeIsActive()) {
            //motor is assumed to be paired with an encoder
            double power = controller.update(motor.getCurrentPosition());
            motor.setPower(power);
        }
    }
}
