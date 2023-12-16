package org.firstinspires.ftc.teamcode.opMode.auton;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;

public class BlueNearTrapConstants {
    //EVERYTHING IS IN INCHES
    //public static double fieldwidth=144;
    //public static double fieldlength=144;
    public static double STARTX=12;
    public static double STARTY=84;
    public static double START_HEADING=0;
    //this will change based on where beacon is using opencv
    //1 is left stripe 2 is top strip 3 is right strip
    public static int beaconloc=-1;
    public static final Pose2d start = new Pose2d(STARTX, STARTY, START_HEADING);
    //place1locx is the x location where we place the pixel if beacon located at beacon1loc and etc.
    //public static double place1locx=36;
    //public static double place1locy=96;
    //public static double place2locx=48;
    //public static double place2locy=90;
    //public static double place3locx=36;
    //public static double place3locy=72;
    public static double placeRad1=Math.toRadians(90);
    public static double placeRad2=Math.atan2(1,2);
    public static double placeRad3=Math.toRadians(270);
    public static double beaconAreaPlacex=36;
    public static double beaconAreaPlacey=84;
    //public static final Vector2d placelocation1 = new Vector2d(place1locx, place1locy);
    //public static final Vector2d placelocation2 = new Vector2d(place2locx, place2locy);
    //public static final Vector2d placelocation3 = new Vector2d(place3locx, place3locy);
    public static final Pose2d Robot2Beacon= new Pose2d();
    public static double parkplacex=60;
    public static double parkplacey=132;
    public static final Pose2d parkplace = new Pose2d(parkplacex,parkplacey, Math.toRadians(270));
    public static double boardx=36;
    public static double boardy=120;
    public static final Pose2d board = new Pose2d(boardx,boardy, Math.toRadians(90));
}
