package org.firstinspires.ftc.teamcode.hardware;

public class Func {//min is -1 max is 1
    public static double map(double val, double min, double max, double outMin, double outMax) {
        return outMin+((val-min)/(max-min))*(outMax-outMin);
    }
}
