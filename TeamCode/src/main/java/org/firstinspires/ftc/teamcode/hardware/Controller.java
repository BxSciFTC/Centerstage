package org.firstinspires.ftc.teamcode.hardware;

import com.qualcomm.robotcore.hardware.HardwareMap;

//teleOp class
public class Controller implements Mechanism {
    Robot robot = new Robot();

    HardwareMap hwMap;

    @Override
    public void init(HardwareMap hwMap) {
        this.hwMap = hwMap;
        robot.init(hwMap);
    }
}
