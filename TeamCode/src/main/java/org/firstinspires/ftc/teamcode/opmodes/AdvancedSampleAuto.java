package org.firstinspires.ftc.teamcode.opmodes;

import static org.firstinspires.ftc.teamcode.subsystems.CommandGroups.liftHigh;
import static org.firstinspires.ftc.teamcode.subsystems.CommandGroups.retract;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.TranslationalVelConstraint;
import com.acmerobotics.roadrunner.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.roadrunner.SparkFunOTOSDrive;
import org.firstinspires.ftc.teamcode.subsystems.CommandGroups;
import org.firstinspires.ftc.teamcode.subsystems.Wavedash;
import org.firstinspires.ftc.teamcode.subsystems.arm;
import org.firstinspires.ftc.teamcode.subsystems.deposit;
import org.firstinspires.ftc.teamcode.subsystems.extendo;
import org.firstinspires.ftc.teamcode.subsystems.intake;
import org.firstinspires.ftc.teamcode.subsystems.lift;
import org.firstinspires.ftc.teamcode.util.LoopTimes;
import org.firstinspires.ftc.teamcode.util.MercurialAction;
import org.firstinspires.ftc.teamcode.util.SilkRoad;

import dev.frozenmilk.dairy.core.util.features.BulkRead;
import dev.frozenmilk.mercurial.Mercurial;
import dev.frozenmilk.mercurial.commands.groups.Parallel;
import dev.frozenmilk.mercurial.commands.groups.Sequential;
import dev.frozenmilk.mercurial.commands.util.Wait;

@Autonomous
@Mercurial.Attach
@arm.Attach
@deposit.Attach
@intake.Attach
@lift.Attach
@extendo.Attach
@Wavedash.Attach
@BulkRead.Attach
@LoopTimes.Attach
public class AdvancedSampleAuto extends OpMode {
    private final Pose2d initialPose = new Pose2d(-24, -62.5, Math.toRadians(90));
    private Action driveAction;
    private Action intakeSample(){
        return new MercurialAction(new Sequential(CommandGroups.intake(), new Wait(.25), intake.closeClaw(), new Wait(.25), retract(), liftHigh()));
    }

    @Override
    public void init() {
        driveAction = Wavedash.actionBuilder(initialPose)
                .afterDisp(0, new MercurialAction(liftHigh()))
                .splineTo(new Vector2d(-58, -56), Math.toRadians(230))
                .stopAndAdd(new MercurialAction(deposit.openClaw()))
                .waitSeconds(.2)
                .setReversed(true)
                .splineTo(new Vector2d(-55, -50.5),  Math.toRadians(257-180))
                .stopAndAdd(intakeSample())
                .waitSeconds(4.2)
                .setReversed(false)
                .setTangent(Math.toRadians(200))
                .strafeToLinearHeading(new Vector2d(-58, -54), Math.toRadians(230))
                .stopAndAdd(new MercurialAction(new Parallel(deposit.openClaw())))
                .waitSeconds(.2)
                .strafeToLinearHeading(new Vector2d(-55, -50.5),  Math.toRadians(282))
                .stopAndAdd(intakeSample())
                .waitSeconds(4.2)
                .setTangent(Math.toRadians(200))
                .strafeToLinearHeading(new Vector2d(-58, -54), Math.toRadians(230))
                .stopAndAdd(new MercurialAction(new Parallel(deposit.openClaw())))
                .waitSeconds(.2)
                .afterTime(1, new MercurialAction(lift.goTo(0)))
                .setReversed(true)
                .splineToLinearHeading(new Pose2d(-46, -381, Math.toRadians(-30)), Math.toRadians(-30))
                .stopAndAdd(intakeSample())
                .waitSeconds(3.8)
                .setReversed(false)
                .setTangent(200)
                .strafeToLinearHeading(new Vector2d(-58, -54), Math.toRadians(230))
                .stopAndAdd(new MercurialAction(deposit.openClaw()))
                .waitSeconds(.2)
                .afterTime(1, new MercurialAction(lift.goTo(350)))
                .setReversed(true)
                .setTangent(Math.toRadians(45))
                .splineToLinearHeading(new Pose2d(-24, -9, Math.toRadians(0)), Math.toRadians(0))
                .build();
    }
    @Override
    public void start(){
        Wavedash.runAsync(driveAction);
    }
    @Override
    public void loop() {

    }
}
;