package org.firstinspires.ftc.teamcode.opMode.testing.armTesting;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import static java.lang.Math.pow;

public class ArmMapper3 {
    HardwareMap hwMap;
    NewArm2 arm;
    ElapsedTime pidTime;

    public double shoulderAngle;
    public double elbowAngle;

    public void init(HardwareMap hwMap) {
        this.hwMap = hwMap;
        arm = new NewArm2();
        arm.init(hwMap);
        pidTime = new ElapsedTime();
        busy = false;
    }

    public static double maxA = 10;
    public static double maxV = 45;
    public static double shoulderDx = 0;
    public static double elbowDx = 0;
    boolean busy;

    public void moveToMotionProfileByAngle(double q1, double q2) {
//        if (busy) return;
        busy = true;
        shoulderAngle = q1;
        elbowAngle = q2;

        double shoulderCurrent = arm.shoulderDegrees();
        double elbowCurrent = arm.elbowDegrees();

        double shoulderDistanceDelta = shoulderAngle - shoulderCurrent;
        double elbowDistanceDelta = elbowAngle - elbowCurrent;

        ElapsedTime shoulderTimer = new ElapsedTime();
        ElapsedTime elbowTimer = new ElapsedTime();

//        double shoulderDx = 0;
//        double elbowDx = 0;

        while ((shoulderDx != shoulderDistanceDelta) || (elbowDx != elbowDistanceDelta)) {
            shoulderDx = motion_profile(maxA, maxV, shoulderDistanceDelta, shoulderTimer);
            elbowDx = motion_profile(maxA, maxV, elbowDistanceDelta, elbowTimer);

            arm.shoulderGoToAngle(shoulderCurrent + shoulderDx);
            arm.elbowGoToAngle(elbowCurrent + elbowDx);
        }
        busy = false;
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
    public void PIDUpdate() {
        arm.PIDUpdate();
    }
}
