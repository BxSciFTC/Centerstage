package org.firstinspires.ftc.teamcode.hardware;

import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.teamcode.opMode.teleOp.TeleOpMain;

//teleOp class
public class Controller implements Mechanism {
    Robot robot = new Robot();
//    ElapsedTime timer;

    HardwareMap hwMap;

    //1 is current gamepad, 2 is previous iteration
    private Gamepad gamepadFirst1 = new Gamepad();

    private Gamepad gamepadFirst2 = new Gamepad();

    private Gamepad gamepadSecond1 = new Gamepad();

    private Gamepad gamepadSecond2 = new Gamepad();
    public double frequency = 20;

    @Override
    public void init(HardwareMap hwMap) {
        this.hwMap = hwMap;
        robot.init(hwMap);
//        timer = new ElapsedTime();
    }

    public void run(Gamepad gamepad1, Gamepad gamepad2) {
        gamepadFirst2.copy(gamepadFirst1);
        gamepadFirst1.copy(gamepad1);

        gamepadSecond2.copy(gamepadSecond1);
        gamepadSecond1.copy(gamepad2);

        move(gamepad1);
        planeFly();
        lift();
//        clawHingeMove();
        clawMove();


//        double frequencyTime = 1000.0 / frequency;
//        if (timer.milliseconds() > frequencyTime) {
//            armMove();
//        } else {
//            timer.reset();
//        }

        robot.update();
    }

    void lift() {
        if(gamepadSecond1.a) {
            robot.lift.liftState= Lift.LiftState.DOWN;
        }else if (gamepadSecond1.y){
            robot.lift.liftState= Lift.LiftState.UP;
        }else{
            robot.lift.liftState= Lift.LiftState.NORMAL;
        }
        if (gamepadSecond1.b && !gamepadSecond2.b) {
            robot.lift.liftServoState = Lift.LiftServoState.UP;
        } else if (gamepadSecond1.x && !gamepadSecond2.x) {
            robot.lift.liftServoState = Lift.LiftServoState.DOWN;
        }
    }
    void planeFly(){
        if (gamepadSecond1.dpad_up && !gamepadSecond2.dpad_up){
            robot.plane.fire();
        }
    }

    void armMove() {
        //updownleftright
        if (gamepadFirst1.triangle && !gamepadFirst2.triangle) {
            robot.arm.pickup();
        }
        else if (gamepadFirst1.cross && !gamepadFirst2.cross) {
            robot.arm.rest();
        } else if (gamepadFirst1.square && !gamepadFirst2.square) {
            robot.arm.score1();
        }else if (gamepadFirst1.circle && !gamepadFirst2.circle) {
            robot.arm.score2();
        }

//        double xShift = 0;
//        double yShift = 0;
//
//        //PRESETS
//        if (gamepadFirst1.triangle && !gamepadFirst2.triangle) {
//            yShift += 0.2;
//        }
//        else if (gamepadFirst1.cross && !gamepadFirst2.cross) {
//            yShift -= 0.2;
//        } else if (gamepadFirst1.square && !gamepadFirst2.square) {
//            xShift -= 0.2;
//        }else if (gamepadFirst1.circle && !gamepadFirst2.circle) {
//            xShift += 0.2;
//        }
//        robot.arm.shift(xShift, yShift);
    }

    void clawMove(){ //controls open and close with buttons
//        TeleOpMain.tele.addData("claw move", "x");
        if(gamepadFirst1.left_bumper){
            robot.claw.leftOpen();
//            TeleOpMain.tele.addData("claw move open", "y");
        }
        else{
            robot.claw.leftClose();
//            TeleOpMain.tele.addData("claw close", "z");
        }
        if(gamepadFirst1.right_bumper){
            robot.claw.rightOpen();
        }
        else{
            robot.claw.rightClose();
        }
        robot.claw.update();
//        TeleOpMain.tele.update();
    }

    void move(Gamepad gamepad1) {
        robot.drive.setWeightedDrivePower(gamepad1);
    }
}






//package org.firstinspires.ftc.teamcode.hardware;
//
//        import com.qualcomm.robotcore.hardware.Gamepad;
//        import com.qualcomm.robotcore.hardware.HardwareMap;
//        import com.qualcomm.robotcore.util.ElapsedTime;
//
////teleOp class
//public class Controller implements Mechanism {
//    Robot robot = new Robot();
//    ElapsedTime timer;
//
//    HardwareMap hwMap;
//
//    //1 is current gamepad, 2 is previous iteration
//    private Gamepad gamepadFirst1 = new Gamepad();
//
//    private Gamepad gamepadFirst2 = new Gamepad();
//
//    private Gamepad gamepadSecond1 = new Gamepad();
//
//    private Gamepad gamepadSecond2 = new Gamepad();
//    public double frequency = 20;
//
//    @Override
//    public void init(HardwareMap hwMap) {
//        this.hwMap = hwMap;
//        robot.init(hwMap);
//        timer = new ElapsedTime();
//    }
//
//    public void run(Gamepad gamepad1, Gamepad gamepad2) {
//        gamepadFirst2.copy(gamepadFirst1);
//        gamepadFirst1.copy(gamepad1);
//
//        gamepadSecond2.copy(gamepadSecond1);
//        gamepadSecond1.copy(gamepad2);
//
//        move();
//        planeFly();
//        lift();
//        clawHingeMove();
//        clawMove();
//
//
//        double frequencyTime = 1000.0 / frequency;
//        if (timer.milliseconds() > frequencyTime) {
//            armMove();
//        } else {
//            timer.reset();
//        }
//
//        robot.update();
//    }
//
//    void lift() {
//        if(gamepadFirst1.left_trigger>0.1) {
//            robot.lift.liftState= Lift.LiftState.DOWN;
//        }else if (gamepadFirst1.right_trigger>0.1){
//            robot.lift.liftState= Lift.LiftState.UP;
//        }else{
//            robot.lift.liftState= Lift.LiftState.NORMAL;
//        }
//        if (gamepadFirst1.dpad_left && !gamepadFirst2.dpad_left) {
//            robot.lift.liftServoState = Lift.LiftServoState.UP;
//        } else if (gamepadFirst1.dpad_right && !gamepadFirst2.dpad_right) {
//            robot.lift.liftServoState = Lift.LiftServoState.DOWN;
//        }
//    }
//    void planeFly(){
//        if (gamepadFirst1.touchpad){
//            robot.plane.fire();
//        }
//    }
//
//    void armMove() {
//        double xShift = 0;
//        double yShift = 0;
//
//        if (gamepadFirst1.dpad_up && !gamepadFirst2.dpad_up) {
//            yShift += 0.2;
//        }
//        else if (gamepadFirst1.dpad_down && !gamepadFirst2.dpad_down) {
//            yShift -= 0.2;
//        } else if (gamepadFirst1.dpad_left && !gamepadFirst2.dpad_left) {
//            xShift -= 0.2;
//        }else if (gamepadFirst1.dpad_right && !gamepadFirst2.dpad_right) {
//            xShift += 0.2;
//        }
//        robot.arm.shift(xShift, yShift);
//    }
//
//    void clawHingeMove(){ //controls hinge with buttons
//        if(gamepadFirst1.square && !gamepadFirst2.square){
//            robot.claw.setThirtyAngle();
//        }
//        if(gamepadFirst1.circle && !gamepadFirst2.circle){
//            robot.claw.setZeroAngle();
//        }
//        robot.claw.update();
//    }
//    void clawMove(){ //controls open and close with buttons
//        if(gamepadFirst1.right_bumper && !gamepadFirst2.right_bumper){
//            robot.claw.open();
//        }
//        else{
//            robot.claw.close();
//        }
//        robot.claw.update();
//    }
//
//    void move() {
//        robot.drive.setWeightedDrivePower(gamepadFirst1);
//    }
//}
