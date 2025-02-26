package org.firstinspires.ftc.teamcode.lastYearCode;

import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Controls {

    public MecanumDriveOld drive;

    //1 is current gamepad, 2 is previous iteration
    private Gamepad gamepad1 = new Gamepad();

    private Gamepad gamepad2 = new Gamepad();

    public void init(HardwareMap hwMap) {
        drive = new MecanumDriveOld(hwMap);
    }

    public void run(Gamepad gamepad) {
        gamepad2.copy(gamepad1);
        gamepad1.copy(gamepad);
        move();
    }

    private void move() {
        drive.setWeightedDrivePower(gamepad1);

    }
}
