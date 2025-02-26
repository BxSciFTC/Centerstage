package com.example.meepmeep;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

public class MeepMeepTesting {
    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(900);

        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(60, 60, Math.toRadians(180), Math.toRadians(180), 15)
                .followTrajectorySequence(drive ->
                        drive.trajectorySequenceBuilder(new Pose2d(11, -61, Math.toRadians(90)))
                                .lineToSplineHeading(new Pose2d(11, -52, Math.toRadians(110)))
                                .UNSTABLE_addTemporalMarkerOffset(0, () -> {

                                })
                                .waitSeconds(1)
                                .lineToLinearHeading(new Pose2d(39, -48, Math.toRadians(0)))
                                .setTangent(90)
                                .splineToSplineHeading(new Pose2d(39, -34, Math.toRadians(0)), Math.toRadians(0))
                                .UNSTABLE_addTemporalMarkerOffset(0, () -> {

                                })
                                .waitSeconds(1)
                                .lineToLinearHeading(new Pose2d(50, -57, Math.toRadians(180)))
                                .build()
                );

        meepMeep.setBackground(MeepMeep.Background.FIELD_CENTERSTAGE_JUICE_DARK)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }
}