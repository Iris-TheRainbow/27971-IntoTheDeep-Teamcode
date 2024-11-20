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
                .strafeTo(new Vector2d(-24, -56))
                .splineTo(new Vector2d(-51, -57), Math.toRadians(210))
                        .waitSeconds(3)
                        .setReversed(true)
                        .splineTo(new Vector2d(-36, -48), Math.toRadians(0))
                        .setReversed(false)
                        .splineTo(new Vector2d( -48, -40), Math.toRadians(90))
                        .waitSeconds(2)
                        .setReversed(true)
                        .splineTo(new Vector2d(-36, -48), Math.toRadians(0))
                        .setReversed(false)
                        .splineTo(new Vector2d(-54, -54), Math.toRadians(225))
                        .waitSeconds(2)
                        .setReversed(true)
                        .splineTo(new Vector2d(-36, -48), Math.toRadians(0))
                        .setReversed(false)

                        .splineTo(new Vector2d(-26, -12), Math.toRadians(0))
                        .build());

        meepMeep.setBackground(MeepMeep.Background.FIELD_INTO_THE_DEEP_JUICE_DARK)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }
}