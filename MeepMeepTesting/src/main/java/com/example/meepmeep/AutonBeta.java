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
                .followTrajectorySequence(drive -> drive.trajectorySequenceBuilder(new Pose2d(-33, -63, Math.toRadians(0)))
                        .setReversed(true)
                        .splineToConstantHeading(new Vector2d(-50, -36), Math.toRadians(90))
                        .waitSeconds(1)
                        //brk
                        .setReversed(false)
                        .splineTo(new Vector2d(-23,-46), Math.toRadians(270))
                        .splineToLinearHeading(new Pose2d(-48, -48, Math.toRadians(225)), Math.toRadians(150))
                        .waitSeconds(5)
                        .splineToLinearHeading(new Pose2d(-48, -36, Math.toRadians(135)), Math.toRadians(135))
                        .waitSeconds(5)
                        .splineToLinearHeading(new Pose2d(-48, -48, Math.toRadians(225)), Math.toRadians(225))
                        .waitSeconds(5)
                        .splineToLinearHeading(new Pose2d(-56, -25, Math.toRadians(180)), Math.toRadians(180))
                        .waitSeconds(5)
                        .setReversed(true)
                        .splineToLinearHeading(new Pose2d(-48, -48, Math.toRadians(225)), Math.toRadians(225))
                        .build());


        meepMeep.setBackground(MeepMeep.Background.FIELD_INTOTHEDEEP_JUICE_DARK)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }
}