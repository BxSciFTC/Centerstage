package org.firstinspires.ftc.teamcode.opMode.testing.armTesting;

import com.arcrobotics.ftclib.util.InterpLUT;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.teamcode.hardware.Mechanism;
import org.firstinspires.ftc.teamcode.hardware.NewArm;
import org.firstinspires.ftc.teamcode.hardware.RobotConstants;

import static java.lang.Math.*;


//formulaic mapping of motors and angles to x y coordinate
public class ArmMapper2 implements Mechanism {

    public NewArm2 arm;
    HardwareMap hwMap;

    //is q1 angle in the model
    public double shoulderAngle;
    public double elbowAngle;


    //is q2 angle in the model, i.e. acute angle between 2 lengths
    public double elbowDownAngle;

    public static double xPos, yPos;
    ElapsedTime pidTime;


    @Override
    public void init(HardwareMap hwMap) {
        this.hwMap = hwMap;
        arm = new NewArm2();
        arm.init(hwMap);
        pidTime = new ElapsedTime();

        xPos = 0;
        yPos = 20;
    }
    /*
        Forward is positive x
        Up is positive y
    */

    public void moveTo(double x1, double y1) {
        double[] angles = calculateAngle(x1, y1);

//        if (strainedAngles(angles)) return;

        shoulderAngle = angles[0];
        elbowAngle = angles[1];
        elbowDownAngle = angles[2];

        this.xPos = x1;
        this.yPos = y1;

        arm.shoulderGoToAngle(shoulderAngle);
        arm.elbowGoToAngle(elbowAngle);
    }

    public void moveToInterpolated(double x1, double y1) {
        double[] angles = calculateAngle(x1, y1);
        shoulderAngle = angles[0];
        elbowAngle = angles[1];
        elbowDownAngle = angles[2];

        //shoulder Look up table
        InterpLUT sInterp = new InterpLUT();
        //elbow Look up table
        InterpLUT eInterp = new InterpLUT();
        sInterp.add(0, arm.shoulderDegrees());
        sInterp.add(1, shoulderAngle);
        eInterp.add(0, arm.elbowDegrees());
        eInterp.add(1, elbowAngle);

        pidTime.reset();
        double time = 2000;
        while (pidTime.milliseconds() <= time) {
            arm.shoulderGoToAngle(sInterp.get(pidTime.milliseconds()/time));
            arm.elbowGoToAngle(eInterp.get(pidTime.milliseconds()/time));
        }
    }

    public void moveToMotionProfile(double x1, double y1) {
        double[] angles = calculateAngle(x1, y1);
        shoulderAngle = angles[0];
        elbowAngle = angles[1];
        elbowDownAngle = angles[2];

        double shoulderCurrent = arm.shoulderDegrees();
        double elbowCurrent = arm.elbowDegrees();

        double shoulderDistanceDelta = shoulderAngle - shoulderCurrent;
        double elbowDistanceDelta = elbowAngle - elbowCurrent;

        ElapsedTime shoulderTimer = new ElapsedTime();
        ElapsedTime elbowTimer = new ElapsedTime();
        double maxA = 1;
        double maxV = 1;

        double shoulderDx = 0;
        double elbowDx = 0;

        while (!(shoulderDx != shoulderDistanceDelta) || !(elbowDx != elbowDistanceDelta)) {
            shoulderDx = motion_profile(maxA, maxV, shoulderDistanceDelta, shoulderTimer);
            elbowDx = motion_profile(maxA, maxV, elbowDistanceDelta, elbowTimer);

            arm.shoulderGoToAngle(shoulderCurrent + shoulderDx);
            arm.elbowGoToAngle(elbowCurrent + elbowDx);
        }
    }

    double motion_profile(double max_acceleration, double max_velocity, double distance, ElapsedTime elapsed_time) {

        //Return the current reference position based on the given motion profile times, maximum acceleration, velocity, and current time.

        // Calculate the time it takes to accelerate to max velocity
        double acceleration_dt = max_velocity / max_acceleration;

        // If we can't accelerate to max velocity in the given distance, we'll accelerate as much as possible
        double halfway_distance = distance / 2;
        double acceleration_distance = 0.5 * max_acceleration * pow(acceleration_dt, 2);

        if (acceleration_distance > halfway_distance) {
            acceleration_dt = Math.sqrt(halfway_distance / (0.5 * max_acceleration));
        }

        acceleration_distance = 0.5 * max_acceleration * pow(acceleration_dt, 2);

        // recalculate max velocity based on the time we have to accelerate and decelerate
        max_velocity = max_acceleration * acceleration_dt;

        // we decelerate at the same rate as we accelerate
        double deceleration_dt = acceleration_dt;

        // calculate the time that we're at max velocity
        double cruise_distance = distance - 2 * acceleration_distance;
        double cruise_dt = cruise_distance / max_velocity;
        double deceleration_time = acceleration_dt + cruise_dt;

        // check if we're still in the motion profile
        double entire_dt = acceleration_dt + cruise_dt + deceleration_dt;
        if (elapsed_time.seconds() > entire_dt) {
            return distance;
        }

        // if we're accelerating
        if (elapsed_time.seconds() < acceleration_dt) {
            // use the kinematic equation for acceleration
            return 0.5 * max_acceleration * pow(elapsed_time.seconds(), 2);
        }

        // if we're cruising
        else if (elapsed_time.seconds() < deceleration_time) {
            acceleration_distance = 0.5 * max_acceleration * pow(acceleration_dt, 2);
            double cruise_current_dt = elapsed_time.seconds() - acceleration_dt;

            // use the kinematic equation for constant velocity
            return acceleration_distance + max_velocity * cruise_current_dt;
        }

        // if we're decelerating
        else {
            acceleration_distance = 0.5 * max_acceleration * pow(acceleration_dt, 2);
            cruise_distance = max_velocity * cruise_dt;
            deceleration_time = elapsed_time.seconds() - deceleration_time;

            // use the kinematic equations to calculate the instantaneous desired position
            return acceleration_distance + cruise_distance + max_velocity * deceleration_time - 0.5 * max_acceleration * pow(deceleration_time, 2);
        }
    }





    public void shift(double x1, double y1) {
        double newX = isInRange(xPos + x1, yPos) ? xPos + x1 : xPos;
        double newY = isInRange(newX, yPos + y1) ? yPos + y1 : yPos;

        this.xPos = newX;
        this.yPos = newY;

        moveTo(newX, newY);
    }

    public void collectPixels1() {

    }

    public void rest() {

    }

    public void score1Spot() {

    }

    public void doThing() {
        this.xPos -= 4;
    }

    //tells if coordinate is outside the range of motion of the two arms(a circle shape)
    public boolean isInRange(double x, double y) {
        double totalLen = RobotConstants.shoulderLen + RobotConstants.elbowLen;
        return !(abs(x) > abs(cos(atan2(y, x))) * totalLen || abs(y) > abs(sin(atan2(y, x)))* totalLen);
    }

//    //we keep q1 bounded 0-90 and 180-q2 bounded 0-180
//    public boolean strainedAngles(double[] angles) {
//        return (angles[0] < 0 || angles[0] > 90 || angles[1] > 180 || angles[1] < 10);
//    }

    public void PIDUpdate() {
        arm.shoulderGoToAngle(shoulderAngle);
        arm.elbowGoToAngle(elbowAngle);
        arm.PIDUpdate();
    }

    //elbow angle will be DOWN angle
    //shoulder angle will be angle of elevation
    public double[] calculateAngle(double x, double y) {
        double a = RobotConstants.shoulderLen;
        double b = RobotConstants.elbowLen;
        double armAngle = -1 * acos(
                (pow(x, 2) + pow(y, 2) - pow(a, 2) - pow(b, 2))/
                        (2 * a * b)
        );
        double shoulderAngle = atan(y / x) +atan (
                (b * sin(armAngle)) /
                        (a + b * cos(armAngle))

        );
        //q1, angle between arms, and q2 in the model
        return new double[] {Math.toDegrees(shoulderAngle), 180-Math.toDegrees(armAngle),Math.toDegrees(armAngle)};
    }
}
