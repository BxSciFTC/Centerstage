package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

//teleOp class
public class Controller implements Mechanism {
    HardwareMap hwMap;
    Claw claw;
    ViperSlide slide;
    Plane plane;
    Lift lift;
    MecanumDrive drive;

    //1 is current gamepad, 2 is previous iteration
    private Gamepad gamepad1 = new Gamepad();

    private Gamepad gamepad2 = new Gamepad();

    ElapsedTime time = new ElapsedTime();

    boolean trigger = false;
    boolean planeOn = false;

    @Override
    public void init(HardwareMap hwMap) {
        this.hwMap = hwMap;
        claw = new Claw();
        slide = new ViperSlide();
        plane = new Plane();
        lift = new Lift();
        drive = new MecanumDrive(hwMap);
        claw.init(hwMap);
        slide.init(hwMap);
        plane.init(hwMap);
        lift.init(hwMap);
    }

    public void run(Gamepad gamepad) {
        gamepad2.copy(gamepad1);
        gamepad1.copy(gamepad);
        slide.update();
        move();
        slideAndClaw();
        lift();
        planeFly();
    }

    void lift(){
        if (gamepad1.dpad_up) {
            lift.setPower(1);
        } else if (gamepad1.dpad_down) {
            lift.setPower(-1);
        } else {
            lift.setPower(0);
        }
    }
    void planeFly(){
        if (gamepad1.square && !gamepad2.square) {
            planeOn = !planeOn;
            if (planeOn)
                plane.on();
            else
                plane.off();
        }
    }

    void slideAndClaw() {
        if (gamepad1.cross  && !gamepad2.cross) {
            slide.low();
            claw.zeroDeg();
            claw.open();
        }
        else if (gamepad1.circle  && !gamepad2.circle) {
            slide.mid();
            claw.sixtyDeg();
        } else if (gamepad1.triangle && !gamepad2.triangle) {
            slide.high();
            claw.sixtyDeg();
        } else if (gamepad1.left_bumper && !gamepad2.left_bumper) {
            claw.close();
        } else if (gamepad1.right_bumper && !gamepad2.right_bumper) {
            claw.open();
        }

        if (gamepad1.left_trigger != 0 || gamepad1.right_trigger != 0) {
            if (trigger == false) time.reset();
            trigger = true;
            if (time.milliseconds() % 100 == 0) {
                slide.shiftSlide((int) (100 * (gamepad1.right_trigger - gamepad1.left_trigger)));
            }
        } else {
            trigger = false;
        }
    }

    void move() {
        drive.setWeightedDrivePower(gamepad1);
    }
}
