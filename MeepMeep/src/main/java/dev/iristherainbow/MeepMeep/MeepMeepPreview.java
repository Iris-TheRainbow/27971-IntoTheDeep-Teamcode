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
                .setConstraints(70, 50, Math.toRadians(180), Math.toRadians(180), 13)
                .build();
        Vector2d specDepo = new Vector2d(0, -34);
        Vector2d specIntkae =  new Vector2d(48,-60);
        myBot.runAction(myBot.getDrive().actionBuilder(new Pose2d(12, -62.5, Math.toRadians(90)))
                .strafeTo(specDepo)
                .setReversed(true)
                .splineToConstantHeading(new Vector2d(33, -35), Math.toRadians(70))
                .splineToConstantHeading(new Vector2d(33, -18), Math.toRadians(70))
                .splineToConstantHeading(new Vector2d(44, -18), Math.toRadians(-35))
                .splineToConstantHeading(new Vector2d(46, -52), Math.toRadians(270))
                .splineToConstantHeading(new Vector2d(48, -14), Math.toRadians(0))
                .splineToConstantHeading(new Vector2d(52, -14), Math.toRadians(-35))
                .splineToConstantHeading(new Vector2d(52, -52), Math.toRadians(270))
                .splineToConstantHeading(new Vector2d(56, -14), Math.toRadians(0))
                .splineToConstantHeading(new Vector2d(57, -14), Math.toRadians(-35))
                .splineToConstantHeading(new Vector2d(60, -52), Math.toRadians(270))
                .strafeToLinearHeading(specIntkae, Math.toRadians(270))
                .strafeToLinearHeading(specDepo, Math.toRadians(90))
                .strafeToLinearHeading(specIntkae, Math.toRadians(270))
                .strafeToLinearHeading(specDepo, Math.toRadians(90))
                .strafeToLinearHeading(specIntkae, Math.toRadians(270))
                .strafeToLinearHeading(specDepo, Math.toRadians(90))
                .setReversed(true)
                .setTangent(Math.toRadians(-45))
                .splineTo(new Vector2d(32,-48), Math.toRadians(-5))
                .build());
        /*myBot.runAction(myBot.getDrive().actionBuilder(new Pose2d((-45+15.5/2), -62.5, Math.toRadians(180)))
                .strafeToLinearHeading(new Vector2d(-58, -56), Math.toRadians(230))
                .waitSeconds(.3)
                .setReversed(true)
                .strafeToLinearHeading(new Vector2d(-52, -50.5),  Math.toRadians(257))
                        .waitSeconds(2)
                .setReversed(false)
                .strafeToLinearHeading(new Vector2d(-56, -54), Math.toRadians(230))
                .waitSeconds(.2)
                .strafeToLinearHeading(new Vector2d(-55, -50.5),  Math.toRadians(270))
                        .waitSeconds(2)
                .setTangent(Math.toRadians(200))
                .strafeToLinearHeading(new Vector2d(-57, -55), Math.toRadians(230))
                .waitSeconds(1)
                .setReversed(true)
                .strafeToLinearHeading(new Vector2d(-42.5, -39), Math.toRadians(-25))
                        .waitSeconds(2)
                .setReversed(false)
                .setTangent(200)
                .strafeToLinearHeading(new Vector2d(-58, -54), Math.toRadians(230))
//                .waitSeconds(.2)
//                .setReversed(true)
//                .setTangent(Math.toRadians(45))
//                .splineTo(new Vector2d(-24, -9), Math.toRadians(0))
//                .stopAndAdd(intakeSample())
//                .setReversed(false)
//                .splineTo(new Vector2d(-58, -54), Math.toRadians(230))
//                .stopAndAdd(new MercurialAction(deposit.openClaw()))
                .waitSeconds(.2)
                .setReversed(true)
                .setTangent(Math.toRadians(45))
                .splineToLinearHeading(new Pose2d(-24, -9, Math.toRadians(0)), Math.toRadians(0))
                .build());
*/
        meepMeep.setBackground(MeepMeep.Background.FIELD_INTO_THE_DEEP_JUICE_DARK)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }
}