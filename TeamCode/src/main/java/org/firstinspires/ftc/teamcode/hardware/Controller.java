package org.firstinspires.ftc.teamcode.hardware;

import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

//teleOp class
public class Controller implements Mechanism {
    Robot robot = new Robot();
    ElapsedTime timer;

    HardwareMap hwMap;

    //1 is current gamepad, 2 is previous iteration
    private Gamepad gamepad1 = new Gamepad();

    private Gamepad gamepad2 = new Gamepad();
    public double frequency = 20;

    @Override
    public void init(HardwareMap hwMap) {
        this.hwMap = hwMap;
        robot.init(hwMap);
        timer = new ElapsedTime();
    }

    public void run(Gamepad gamepad) {
        gamepad2.copy(gamepad1);
        gamepad1.copy(gamepad);
        move();
        planeFly();
        lift();
        clawHingeMove();
        clawMove();


        double frequencyTime = 1000.0 / frequency;
        if (timer.milliseconds() > frequencyTime) {
            armMove();
        } else {
            timer.reset();
        }

        robot.update();
    }

    void lift() {
        if(gamepad1.left_trigger>0.1) {
            robot.lift.liftState= Lift.LiftState.DOWN;
        }else if (gamepad1.right_trigger>0.1){
            robot.lift.liftState= Lift.LiftState.UP;
        }else{
            robot.lift.liftState= Lift.LiftState.NORMAL;
        }
        if (gamepad1.dpad_left && !gamepad2.dpad_left) {
            robot.lift.liftServoState = Lift.LiftServoState.UP;
        } else if (gamepad1.dpad_right && !gamepad2.dpad_right) {
            robot.lift.liftServoState = Lift.LiftServoState.DOWN;
        }
    }
    void planeFly(){
        if (gamepad1.touchpad){
            robot.plane.fire();
        }
    }

    void armMove() {
        double xShift = 0;
        double yShift = 0;

        if (gamepad1.dpad_up && !gamepad2.dpad_up) {
            yShift += 0.2;
        }
        else if (gamepad1.dpad_down && !gamepad2.dpad_down) {
            yShift -= 0.2;
        } else if (gamepad1.dpad_left && !gamepad2.dpad_left) {
            xShift -= 0.2;
        }else if (gamepad1.dpad_right && !gamepad2.dpad_right) {
            xShift += 0.2;
        }
        robot.arm.shift(xShift, yShift);
    }

    void clawHingeMove(){ //controls hinge with buttons
        if(gamepad1.square && !gamepad2.square){
            robot.claw.setThirtyAngle();
        }
        if(gamepad1.circle && !gamepad2.circle){
            robot.claw.setZeroAngle();
        }
        robot.claw.update();
    }
    void clawMove(){ //controls open and close with buttons
        if(gamepad1.right_bumper && !gamepad2.right_bumper){
            robot.claw.open();
        }
        else{
            robot.claw.close();
        }
        robot.claw.update();
    }

    void move() {
        robot.drive.setWeightedDrivePower(gamepad1);
    }
}
