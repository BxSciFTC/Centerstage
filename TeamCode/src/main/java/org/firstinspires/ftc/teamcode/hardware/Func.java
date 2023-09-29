package org.firstinspires.ftc.teamcode.hardware;

public class Func {
    public static double map(double val, double min, double max, double outMin, double outMax) {
        return outMin+((val-min)/(max-min))*(outMax-outMin);
    }
}
