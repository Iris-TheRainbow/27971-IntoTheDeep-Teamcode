package org.firstinspires.ftc.teamcode.opmodes;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.TranslationalVelConstraint;
import com.acmerobotics.roadrunner.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.roadrunner.SparkFunOTOSDrive;
import org.firstinspires.ftc.teamcode.subsystems.arm;
import org.firstinspires.ftc.teamcode.subsystems.deposit;
import org.firstinspires.ftc.teamcode.subsystems.drive;
import org.firstinspires.ftc.teamcode.subsystems.extendo;
import org.firstinspires.ftc.teamcode.subsystems.intake;
import org.firstinspires.ftc.teamcode.subsystems.lift;
import org.firstinspires.ftc.teamcode.util.LoopTimes;
import org.firstinspires.ftc.teamcode.util.MercurialAction;
import org.firstinspires.ftc.teamcode.util.SilkRoad;

import org.firstinspires.ftc.teamcode.util.BulkRead;
import org.firstinspires.ftc.teamcode.util.SlothFinder;

import dev.frozenmilk.mercurial.Mercurial;
import dev.frozenmilk.mercurial.commands.groups.Parallel;
import dev.frozenmilk.mercurial.commands.groups.Sequential;

@Autonomous
@Mercurial.Attach
@arm.Attach
@deposit.Attach
@intake.Attach
@lift.Attach
@extendo.Attach
@BulkRead.Attach
@LoopTimes.Attach
@SilkRoad.Attach
@SlothFinder.Attach
public class sampleauto extends OpMode {
    private final Pose2d initialPose = new Pose2d(-24, -62.5, Math.toRadians(90));
    private SparkFunOTOSDrive drive;
    private Action driveAction;

    @Override
    public void init() {
        drive = new SparkFunOTOSDrive(hardwareMap, initialPose);
        driveAction = drive.actionBuilder(initialPose)
                .afterDisp(0, new MercurialAction(new Parallel(lift.goTo(2000), deposit.closeClaw(), arm.armUp(), deposit.wristDeposit())))
                .splineTo(new Vector2d(-58, -56), Math.toRadians(230))
                .stopAndAdd(new MercurialAction(new Parallel(deposit.openClaw())))
                .waitSeconds(.5)
                .afterTime(.5, new MercurialAction(lift.goTo(0)))
                .setReversed(true)
                .splineTo(new Vector2d(-55, -51),  Math.toRadians(259-180), new TranslationalVelConstraint(5))
                .afterTime(0, new MercurialAction( new Parallel(extendo.goTo(500), intake.wristIntake(), intake.openClaw())))
                .afterTime(2, new MercurialAction(new Parallel(intake.closeClaw())))
                .afterTime(2.5, new MercurialAction(new Sequential(new Parallel(extendo.goTo(0), lift.goTo(0), intake.wristTransfer(), deposit.wristTransfer(), deposit.openClaw(), arm.armWait()), arm.armTransfer(), deposit.closeClaw(), intake.openClaw(), new Parallel(arm.armUp(), deposit.wristDeposit()), new Parallel(lift.goTo(2000), deposit.closeClaw(), arm.armUp()))))
                .waitSeconds(5)
                .splineTo(new Vector2d(-58, -54), Math.toRadians(230))
                .waitSeconds(.5)
                .stopAndAdd(new MercurialAction(new Parallel(deposit.openClaw())))
                .waitSeconds(.5)
                .strafeToLinearHeading(new Vector2d(-55, -51),  Math.toRadians(282), new TranslationalVelConstraint(5))
                .afterTime(0, new MercurialAction( new Parallel(extendo.goTo(500), intake.wristIntake(), intake.openClaw(), lift.goTo(0))))
                .afterTime(2, new MercurialAction(new Parallel(intake.closeClaw())))
                .afterTime(2.5, new MercurialAction(new Sequential(new Parallel(extendo.goTo(0), lift.goTo(0), intake.wristTransfer(), deposit.wristTransfer(), deposit.openClaw(), arm.armWait()), arm.armTransfer(), deposit.closeClaw(), intake.openClaw(), new Parallel(arm.armUp(), deposit.wristDeposit()), new Parallel(lift.goTo(2000), deposit.closeClaw(), arm.armUp()))))
                .waitSeconds(5)
                .splineTo(new Vector2d(-58, -54), Math.toRadians(230))
                .waitSeconds(.5)
                .stopAndAdd(new MercurialAction(new Parallel(deposit.openClaw())))
                .waitSeconds(.5)
                .setReversed(true)
                .splineTo(new Vector2d(-40, -48), Math.toRadians(0))
                .stopAndAdd(new MercurialAction(lift.goTo(550)))
                .setReversed(false)
                .splineTo(new Vector2d(-24, -12), Math.toRadians(0))
                .build();
    }
    @Override
    public void start(){
        SilkRoad.RunAsync(driveAction);
    }
    @Override
    public void loop() {

    }
}
;