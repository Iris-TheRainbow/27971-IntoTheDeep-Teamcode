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
                .setConstraints(45, 25, Math.toRadians(180), Math.toRadians(180), 13)
                .build();

        myBot.runAction(myBot.getDrive().actionBuilder(new Pose2d(-24, -62.5, Math.toRadians(90)))
                .splineTo(new Vector2d(-58, -56), Math.toRadians(230))
                .waitSeconds(.5)
                .setReversed(true)
                .splineTo(new Vector2d(-55, -51),  Math.toRadians(259-180), new TranslationalVelConstraint(5))
                .setReversed(false)
                .splineTo(new Vector2d(-58, -54), Math.toRadians(230))
                .waitSeconds(.5)
                .waitSeconds(.5)
                .strafeToLinearHeading(new Vector2d(-55, -51),  Math.toRadians(282), new TranslationalVelConstraint(5))
                        .waitSeconds(.01)
                .splineTo(new Vector2d(-58, -54), Math.toRadians(230))
                .waitSeconds(.5)
                .waitSeconds(.5)
                .setReversed(true)
                .splineToLinearHeading(new Pose2d(-46, -38, Math.toRadians(-30)), Math.toRadians(-30))
                        .setReversed(false)
                .splineTo(new Vector2d(-58, -54), Math.toRadians(230))
                .waitSeconds(.5)
                .waitSeconds(.5)
                .setReversed(true)
                        .setTangent(Math.toRadians(45))
                .splineToLinearHeading(new Pose2d(-24, -12, Math.toRadians(0)), Math.toRadians(0))
                .build());

        meepMeep.setBackground(MeepMeep.Background.FIELD_INTO_THE_DEEP_JUICE_DARK)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }
}