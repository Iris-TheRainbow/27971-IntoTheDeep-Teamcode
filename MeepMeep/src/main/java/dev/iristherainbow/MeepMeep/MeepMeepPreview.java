package dev.iristherainbow.MeepMeep;

import com.acmerobotics.roadrunner.Pose2d;
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
                .setConstraints(60, 60, Math.toRadians(180), Math.toRadians(180), 15)
                .build();

        myBot.runAction(myBot.getDrive().actionBuilder(new Pose2d(-24, -62.5, Math.toRadians(90)))
                // start lift raise
                .strafeToLinearHeading(new Vector2d(-58, -56), Math.toRadians(230))
                        .waitSeconds(1)
                        // deposit
                        // retract lift
                        .turnTo(Math.toRadians(245))
                        // extend
                        // grab
                        // retract extendo
                        .waitSeconds(2)
                        // raise lift
                        .turnTo(Math.toRadians(230))
                        // deposit
                        // retract lift
                        .waitSeconds(2)
                        // extend
                        .turnTo(Math.toRadians(270))
                        // grab
                        // retract extendo
                        .waitSeconds(2)
                        // lift
                        .turnTo(Math.toRadians(230))
                        // deposit
                        // retract lift
                        .waitSeconds(2)
                        // back up
                        .setReversed(true)
                        .splineTo(new Vector2d(-40, -48), Math.toRadians(0))
                        .setReversed(false)
                        .splineTo(new Vector2d(-24, -12), Math.toRadians(0))
                        // raise lift to l1
                        .waitSeconds(1)
                        .strafeTo(new Vector2d(-24, -12))
                .build());

        meepMeep.setBackground(MeepMeep.Background.FIELD_INTO_THE_DEEP_JUICE_DARK)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }
}