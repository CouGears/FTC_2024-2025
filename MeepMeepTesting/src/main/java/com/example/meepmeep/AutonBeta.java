package com.example.meepmeep;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;

import org.rowlandhall.meepmeep.MeepMeep;
import org.rowlandhall.meepmeep.roadrunner.DefaultBotBuilder;
import org.rowlandhall.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

public class AutonBeta {
    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(800);
        double width = 18;
        double height= 18;
        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(40, 40, Math.toRadians(180), Math.toRadians(180), 12.785)
                .setDimensions(width,height)
                .followTrajectorySequence(drive -> drive.trajectorySequenceBuilder(new Pose2d(9.0, -63.0, Math.toRadians(0.0)))
                        .setReversed(false)
                        .setTangent(Math.toRadians(90))
                        .splineToLinearHeading(new Pose2d(0.0, -34.0, Math.toRadians(0)), Math.toRadians(90))
                        .build());


        meepMeep.setBackground(MeepMeep.Background.FIELD_INTOTHEDEEP_JUICE_DARK)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }
}