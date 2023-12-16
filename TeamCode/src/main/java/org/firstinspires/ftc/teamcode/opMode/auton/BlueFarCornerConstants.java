package org.firstinspires.ftc.teamcode.opMode.auton;


import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;

public class BlueFarCornerConstants {
    //EVERYTHING IS IN INCHES
    public static double fieldwidth=144;
    public static double fieldlength=144;
    public static double STARTFARX=12;
    public static double STARTFARY=36;
    //this will change based on where beacon is using opencv
    //1 is left stripe 2 is top strip 3 is right strip
    public static int beaconloc=-1;

    //place1locx is the x location where we place the pixel if beacon located at beacon1loc and etc.
    public static double place1locx=36;
    public static double place1locy=48;
    public static double place2locx=48;
    public static double place2locy=42;
    public static double place3locx=36;
    public static double place3locy=24;
    public static double placeDeg1=90;
    public static double placeDeg2=Math.toDegrees(Math.atan2(1,2));
    public static double placeDeg3=270;
    public static double beaconAreaPlacey=36;
    public static final Vector2d placelocation1 = new Vector2d(place1locx, place1locy);
    public static final Vector2d placelocation2 = new Vector2d(place2locx, place2locy);
    public static final Vector2d placelocation3 = new Vector2d(place3locx, place3locy);
    public static final Pose2d Robot2Beacon= new Pose2d();
    public static double parkplacex=12;
    public static double parkplacey=132;
    public static final Vector2d parkplace = new Vector2d(parkplacex,parkplacey);
    public static double boardx=12;
    public static double boardy=132;
    public static final Pose2d board = new Pose2d(boardx,boardy, Math.toRadians(90));

}
