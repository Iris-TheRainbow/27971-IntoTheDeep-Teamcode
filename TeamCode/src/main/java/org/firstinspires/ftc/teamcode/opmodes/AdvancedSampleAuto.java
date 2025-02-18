package org.firstinspires.ftc.teamcode.opmodes;

import static org.firstinspires.ftc.teamcode.subsystems.CommandGroups.intakeAuto;
import static org.firstinspires.ftc.teamcode.subsystems.CommandGroups.intakeAutoShort;
import static org.firstinspires.ftc.teamcode.subsystems.CommandGroups.liftHigh;
import static org.firstinspires.ftc.teamcode.subsystems.CommandGroups.prepDepo;
import static org.firstinspires.ftc.teamcode.subsystems.CommandGroups.retract;
import static org.firstinspires.ftc.teamcode.subsystems.CommandGroups.transfer;

import androidx.annotation.NonNull;
import androidx.annotation.Nullable;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.IdentityPoseMap;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.TranslationalVelConstraint;
import com.acmerobotics.roadrunner.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.teamcode.roadrunner.SparkFunOTOSDrive;
import org.firstinspires.ftc.teamcode.roadrunner.TrajectoryCommandBuilder;
import org.firstinspires.ftc.teamcode.roadrunner.WavedashMecanumDrive;
import org.firstinspires.ftc.teamcode.subsystems.CommandGroups;
import org.firstinspires.ftc.teamcode.subsystems.Wavedash;
import org.firstinspires.ftc.teamcode.subsystems.arm;
import org.firstinspires.ftc.teamcode.subsystems.deposit;
import org.firstinspires.ftc.teamcode.subsystems.extendo;
import org.firstinspires.ftc.teamcode.subsystems.intake;
import org.firstinspires.ftc.teamcode.subsystems.lift;
import org.firstinspires.ftc.teamcode.util.LoopTimes;
import org.firstinspires.ftc.teamcode.util.MercurialAction;
import org.firstinspires.ftc.teamcode.util.PidToPointBuilder;
import org.firstinspires.ftc.teamcode.util.SilkRoad;

import org.firstinspires.ftc.teamcode.util.BulkRead;
import org.firstinspires.ftc.teamcode.util.SlothFinder;
import org.firstinspires.ftc.teamcode.util.Telem;

import java.util.Set;
import java.util.function.Consumer;
import java.util.function.Supplier;

import dev.frozenmilk.dairy.core.wrapper.Wrapper;
import dev.frozenmilk.mercurial.Mercurial;
import dev.frozenmilk.mercurial.commands.Command;
import dev.frozenmilk.mercurial.commands.Lambda;
import dev.frozenmilk.mercurial.commands.StackUnwinder;
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
@SlothFinder.Attach
@Telem.Attach
public class AdvancedSampleAuto extends OpMode {
    private final Pose2d initialPose = new Pose2d(-36, -62.5, Math.toRadians(180));
    private final Pose2d depositSpot = new Pose2d(-54, -55, Math.toRadians(230));
    private Command driveCommand;
    private Command intakeSample(){
        return new Sequential(CommandGroups.intake(), new Wait(.2), intake.closeClaw(), new Wait(.2));
    }
    private Command intakeSampleShort(){
        return new Sequential(CommandGroups.intakeAutoShort(), new Wait(.2), intake.closeClaw(), new Wait(.45));
    }
    private Command transferAndLift(){
        return new Sequential(retract(), transfer(), liftHigh());
    }

    @Override
    public void init() {
//        TrajectoryCommandBuilder tcb = Wavedash.commandBuilder(initialPose)
//                //.afterDisp(0, (liftHigh()))
//                .strafeToLinearHeading(new Vector2d(-58, -56), Math.toRadians(230))
//                //.stopAndAdd(deposit.openClaw())
//                //.stopAndAdd(new Wait(.3))
//                .setReversed(true)
//                .strafeToLinearHeading(new Vector2d(-52, -50.5),  Math.toRadians(257))
//                //.afterTime(0, extendo.goTo(435))
//                //.stopAndAdd(intakeSample())
//                .setReversed(false)
//                .strafeToLinearHeading(new Vector2d(-56, -54), Math.toRadians(230))
//                //.stopAndAdd(new Parallel(deposit.openClaw()))
//                //.stopAndAdd(new Wait(.3))
//                .setReversed(true)
//                .setTangent(Math.toRadians(35))
//                .strafeToLinearHeading(new Vector2d(-59, -48.5),  Math.toRadians(273))
//                //.stopAndAdd(new Sequential(intakeAutoShort(), new Wait(.25), intake.closeClaw(), new Wait(.25), retract(), transfer(), liftHigh()))
//                .setTangent(Math.toRadians(200))
//                .strafeToLinearHeading(new Vector2d(-57, -55), Math.toRadians(230))
//                //.stopAndAdd(new Wait(1))
//                //.stopAndAdd(new Parallel(deposit.openClaw()))
//                //.afterTime(1, lift.goTo(0))
//                .setReversed(true)
//                .splineToLinearHeading(new Pose2d(-42.5, -39, Math.toRadians(-25)), Math.toRadians(-25))
//                //.stopAndAdd(new Sequential(intakeAuto(), intake.wristRotate(.7), new Wait(.5), intake.closeClaw(), new Wait(.25), intake.wristRotate(.5), retract(), transfer(), liftHigh()))
//                .setReversed(false)
//                .setTangent(200)
//                .strafeToLinearHeading(new Vector2d(-58, -54), Math.toRadians(230))
//                //.stopAndAdd(deposit.openClaw())
////                .waitSeconds(.2)
////                .setReversed(true)
////                .setTangent(Math.toRadians(45))
////                .splineTo(new Vector2d(-24, -9), Math.toRadians(0))
////                .stopAndAdd(intakeSample())
////                .setReversed(false)
////                .splineTo(new Vector2d(-58, -54), Math.toRadians(230))
////                .stopAndAdd(new MercurialAction(deposit.openClaw()))
//                //.stopAndAdd(new Wait(.3))
//                .setReversed(true)
//                .setTangent(Math.toRadians(45))
        //                .splineToLinearHeading(new Pose2d(-28, -9, Math.toRadians(0)), Math.toRadians(0))
//                //.stopAndAdd(lift.goTo(350))
//                .strafeTo(new Vector2d(-20, -9));

        driveCommand = Wavedash.p2pBuilder(initialPose)
                .stopAndAdd(liftHigh())
                .pidTo(depositSpot)
                .duringLast(liftHigh())
                .wait(.5)
                .stopAndAdd(deposit.openClaw())

                //cycle1
                .pidTo(new Pose2d(-45.50, -54, Math.toRadians(90-180)))
                .stopAndAdd(intakeSample())
                .pidTo(new Pose2d(-48, -56, Math.toRadians(240)))
                .duringLast(transferAndLift())
                .pidTo(depositSpot)
                .stopAndAdd(deposit.openClaw())

                //cycle2
                .pidTo(new Pose2d(-55, -50, Math.toRadians(90-180)), 3, 99999)
                .pidTo(new Pose2d(-55, -53, Math.toRadians(90-180)), .5, 2)
                .duringLast(extendo.goTo(465))
                .afterTime(.2, lift.goTo(0))
                .stopAndAdd(intakeSampleShort())
                .pidTo(new Pose2d(-48, -56, Math.toRadians(240)))
                .duringLast(transferAndLift())
                .pidTo(depositSpot)
                .stopAndAdd(deposit.openClaw())

                //cycle3
                .pidTo(new Pose2d(-54, -53, Math.toRadians(230)))
                .pidTo(new Pose2d(-42, -38, Math.toRadians(-25)))
                .afterTime(.25, extendo.goTo(435), lift.goTo(0))
                .stopAndAdd(new Sequential(intakeAuto(), intake.wristRotate(.7), new Wait(.5), intake.closeClaw(), new Wait(.25), intake.wristRotate(.5)))
                .pidTo(new Pose2d(-48, -56, Math.toRadians(240)))
                .duringLast(transferAndLift())
                .pidTo(new Pose2d(-54, -54, Math.toRadians(230)))
                .stopAndAdd(deposit.openClaw())

                //cycle4
                .pidTo(new Pose2d(-44, -13, Math.toRadians(180)), 20, 999) //waypoint so i dont hit the leg
                .pidTo(new Pose2d(-24, -9, Math.toRadians(180)))
                .stopAndAdd(intakeSample())
                .stopAndAdd(intake.wristTransfer())
                .pidTo(new Pose2d(-44, -15, Math.toRadians(180)), 20, 999) //waypoint again
                .stopAndAdd(extendo.goTo(0))
                .pidTo(new Pose2d(-48, -56, Math.toRadians(240)))
                .duringLast(transferAndLift())
                .pidTo(depositSpot)
                .stopAndAdd(deposit.openClaw())

                //park
                .pidTo(new Pose2d(-44, -15, Math.toRadians(0)), 20, 999) //waypoint so I dont hit the leg
                .pidTo(new Pose2d(-22, -9, Math.toRadians(0)))
                .duringLast(lift.goTo(100), deposit.wristSepc())//to touch l1
                .build();
    }
    @Override
    public void start(){
        driveCommand.schedule();
    }

    @Override
    public void loop() {
        telemetry.addLine(Mercurial.INSTANCE.getActiveCommandSnapshot().toString());
    }
}
;