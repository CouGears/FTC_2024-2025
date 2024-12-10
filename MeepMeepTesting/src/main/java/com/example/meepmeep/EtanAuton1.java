package com.example.meepmeep;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;

import org.rowlandhall.meepmeep.MeepMeep;
import org.rowlandhall.meepmeep.roadrunner.DefaultBotBuilder;
import org.rowlandhall.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

public class EtanAuton1 {
    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(800);
        double width = 17.5;
        double height=17.5;
        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(60, 60, Math.toRadians(180), Math.toRadians(180), 15)
                .setDimensions(width,height)
                .followTrajectorySequence(drive -> drive.trajectorySequenceBuilder(new Pose2d(70.3-width/2, -57+height/2, Math.toRadians(90)))
//                        .splineToSplineHeading(new Pose2d(32,-40,Math.toRadians(0)),Math.toRadians(0))
//                        .lineTo(new Vector2d(47-height/2,-24.5))
                        .setReversed(true)
                        .splineToSplineHeading(new Pose2d(47-height/2,-24.5,Math.toRadians(0)),Math.toRadians(0))
//
                        .lineTo(new Vector2d(32,-40))
                        .lineToLinearHeading(new Pose2d(-55,-60,Math.toRadians(180)))

                        .build());


        meepMeep.setBackground(MeepMeep.Background.FIELD_INTOTHEDEEP_JUICE_DARK)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }

}