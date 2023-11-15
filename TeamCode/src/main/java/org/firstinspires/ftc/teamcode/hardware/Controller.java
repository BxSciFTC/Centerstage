package org.firstinspires.ftc.teamcode.hardware;

import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

//teleOp class
public class Controller implements Mechanism {
    Robot robot = new Robot();

    HardwareMap hwMap;

    //1 is current gamepad, 2 is previous iteration
    private Gamepad gamepad1 = new Gamepad();

    private Gamepad gamepad2 = new Gamepad();

    @Override
    public void init(HardwareMap hwMap) {
        this.hwMap = hwMap;
        robot.init(hwMap);
    }

    public void run(Gamepad gamepad) {
        gamepad2.copy(gamepad1);
        gamepad1.copy(gamepad);
        move();
        planeFly();
//        score();
        armMove();
        lift();

        clawHingeMove();
        clawMove();
    }
    void lift(){
        if(gamepad1.left_trigger==0.1) {
            robot.lift.liftState= Lift.LiftState.DOWN;
        }else if (gamepad1.right_trigger==0.1){
            robot.lift.liftState= Lift.LiftState.UP;
        }else{
            robot.lift.liftState= Lift.LiftState.NORMAL;
        }

    }
    void planeFly(){
        if (gamepad1.left_bumper){
            robot.plane.planeState= Plane.PlaneState.ON;
        }else {
            robot.plane.planeState= Plane.PlaneState.OFF;
        }

    }

//    private void move() {
//        drive.setWeightedDrivePower(gamepad1);
//
//    }
    void armMove() {
        double xShift = 0;
        double yShift = 0;

        if (gamepad1.dpad_up) {
            xShift += 1;
        }
        else if (gamepad1.dpad_down) {
            xShift -= 1;
        } else if (gamepad1.triangle) {
            yShift += 1;
        }else if (gamepad1.cross) {
            yShift -= 1;
        }
        robot.armShift(xShift, yShift);
    }

    void clawHingeMove(){ //controls hinge with buttons
        if(gamepad1.square){
            robot.claw.setThirtyAngle();
        }
        if(gamepad1.circle){
            robot.claw.setZeroAngle();
        }
        robot.claw.update();
    }
    void clawMove(){ //controls open and close with buttons
        if(gamepad1.right_bumper){
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
