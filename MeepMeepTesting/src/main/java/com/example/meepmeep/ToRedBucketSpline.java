package com.example.meepmeep;

import com.acmerobotics.roadrunner.geometry.Pose2d;

import org.rowlandhall.meepmeep.MeepMeep;
import org.rowlandhall.meepmeep.roadrunner.DefaultBotBuilder;
import org.rowlandhall.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

public class ToRedBucketSpline {
    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(900);
        double width = 17.5;
        double height=17.5;
        double RStartingPosX = 70-width/2;
        double RStartingPosY = -50-width/2;
        final double RBucketX = -54;
        final double RBucketY = -54;
        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(60, 60, Math.toRadians(180), Math.toRadians(180), 15)
                .setDimensions(width,height)
                .followTrajectorySequence(drive -> drive.trajectorySequenceBuilder(new Pose2d(RStartingPosX, RStartingPosY, Math.toRadians(90)))
                        .lineToLinearHeading(new Pose2d(RStartingPosX-12, RStartingPosY+6, Math.toRadians(180)))
                        .splineToLinearHeading(new Pose2d(RBucketX,RBucketY, Math.toRadians(-135)), Math.toRadians(190))
                        .setReversed(true)
                        .build());


        meepMeep.setBackground(MeepMeep.Background.FIELD_INTOTHEDEEP_OFFICIAL)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }

}