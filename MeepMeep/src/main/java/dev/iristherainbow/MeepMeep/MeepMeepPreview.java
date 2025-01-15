package dev.iristherainbow.MeepMeep;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.TranslationalVelConstraint;
import com.acmerobotics.roadrunner.Vector2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

public class MeepMeepPreview {
    public static void main(String[] args) {
        System.setProperty("sun.java2d.opengl", "true");
        MeepMeep meepMeep = new MeepMeep(800);

        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setDimensions(15.5, 15.5)
                .setConstraints(60, 35, Math.toRadians(180), Math.toRadians(180), 13)
                .build();

        myBot.runAction(myBot.getDrive().actionBuilder(new Pose2d(-24, -62.5, Math.toRadians(90)))
                .splineTo(new Vector2d(-58, -56), Math.toRadians(230))
                .waitSeconds(.2)
                .setReversed(true)
                .splineTo(new Vector2d(-55, -50.5),  Math.toRadians(259-180))
                .waitSeconds(4)
                .setReversed(false)
                .strafeToLinearHeading(new Vector2d(-58, -54), Math.toRadians(230))
                .waitSeconds(.2)
                .strafeToLinearHeading(new Vector2d(-55, -50.5),  Math.toRadians(282))
                .waitSeconds(4)
                .strafeToLinearHeading(new Vector2d(-58, -54), Math.toRadians(230))
                .waitSeconds(.2)
                .setReversed(true)
                .splineToLinearHeading(new Pose2d(-47, -37.7, Math.toRadians(-30)), Math.toRadians(-30))
                .waitSeconds(4.2)
                .setReversed(false)
                .strafeToLinearHeading(new Vector2d(-58, -54), Math.toRadians(230))
                .waitSeconds(.2)
                .setReversed(true)
                .setTangent(Math.toRadians(45))
                .splineToLinearHeading(new Pose2d(-24, -9, Math.toRadians(0)), Math.toRadians(0))
                        .turnTo
                .build());

        meepMeep.setBackground(MeepMeep.Background.FIELD_INTO_THE_DEEP_JUICE_DARK)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }
}