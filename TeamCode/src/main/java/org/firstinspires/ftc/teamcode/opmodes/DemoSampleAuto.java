package org.firstinspires.ftc.teamcode.opmodes;

import static org.firstinspires.ftc.teamcode.subsystems.CommandGroups.intakeAuto;
import static org.firstinspires.ftc.teamcode.subsystems.CommandGroups.liftHigh;
import static org.firstinspires.ftc.teamcode.subsystems.CommandGroups.retract;
import static org.firstinspires.ftc.teamcode.subsystems.CommandGroups.transfer;

import com.acmerobotics.roadrunner.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.subsystems.CommandGroups;
import org.firstinspires.ftc.teamcode.subsystems.Wavedash;
import org.firstinspires.ftc.teamcode.subsystems.arm;
import org.firstinspires.ftc.teamcode.subsystems.deposit;
import org.firstinspires.ftc.teamcode.subsystems.drive;
import org.firstinspires.ftc.teamcode.subsystems.extendo;
import org.firstinspires.ftc.teamcode.subsystems.intake;
import org.firstinspires.ftc.teamcode.subsystems.lift;
import org.firstinspires.ftc.teamcode.subsystems.subAuto;
import org.firstinspires.ftc.teamcode.util.BulkRead;
import org.firstinspires.ftc.teamcode.util.LoopTimes;
import org.firstinspires.ftc.teamcode.util.SlothFinder;
import org.firstinspires.ftc.teamcode.util.Telem;

import dev.frozenmilk.mercurial.Mercurial;
import dev.frozenmilk.mercurial.commands.Command;
import dev.frozenmilk.mercurial.commands.groups.Sequential;
import dev.frozenmilk.mercurial.commands.util.Wait;

@Autonomous
@Mercurial.Attach
@arm.Attach
@deposit.Attach
@intake.Attach
@lift.Attach
@extendo.Attach
@drive.Attach
@BulkRead.Attach
@LoopTimes.Attach
@SlothFinder.Attach
@Telem.Attach
public class DemoSampleAuto extends OpMode {
    private final Pose2d initialPose = new Pose2d(-36, -62.5, Math.toRadians(180));
    private final Pose2d depositSpot = new Pose2d(-53, -55, Math.toRadians(230));
    private Command intakeSample(){
        return new Sequential(CommandGroups.intake(), intake.closeClaw(), new Wait(.15));
    }
    private Command intakeSampleShort(){
        return new Sequential(CommandGroups.intakeAutoShort(), intake.closeClaw(), new Wait(.15));
    }
    private Command transferAndLift(){
        return new Sequential(retract(), transfer(), liftHigh());
    }

    @Override
    public void init() {
    }
    @Override
    public void start(){
        drive.p2p(initialPose)
                .pidTo(depositSpot)
                .duringLast(liftHigh())

                //cycle1
                .pidTo(new Pose2d(-45.50, -54, Math.toRadians(90-180)))
                .duringLast(deposit.openClaw())
                .stopAndAdd(intakeSample())
                .pidTo(depositSpot)
                .duringLast(transferAndLift())
                .stopAndAdd(deposit.openClaw())

                //cycle2
                .pidTo(new Pose2d(-55, -50, Math.toRadians(90-180)), 3, 99999)
                .pidTo(new Pose2d(-55, -53, Math.toRadians(90-180)), .5, 2)
                .duringLast(extendo.goTo(465))
                .afterTime(.2, lift.goTo(0))
                .stopAndAdd(intakeSampleShort())
                .pidTo(depositSpot)
                .duringLast(transferAndLift())

                //cycle3
                .pidTo(new Pose2d(-54, -53, Math.toRadians(230)))
                .duringLast(deposit.openClaw())
                .pidTo(new Pose2d(-42, -38, Math.toRadians(-25)))
                .afterTime(.25, extendo.goTo(435), lift.goTo(0))
                .duringLast(intake.wristRotate(.7))
                .stopAndAdd(new Sequential(intakeAuto(), intake.wristRotate(.7), new Wait(.2), intake.closeClaw(), new Wait(.15), intake.wristRotate(.5)))
                .pidTo(depositSpot)
                .duringLast(transferAndLift())
                .stopAndAdd(deposit.openClaw())
                .stopAndAdd(new Sequential(arm.armWait(), retract()))
                //park
                .pidTo(new Pose2d(-44, -13, Math.toRadians(180)), 20, 999) //waypoint so i dont hit the leg
                .duringLast(new Sequential(new Wait(.1), lift.goTo(100)))
                .pidTo(new Pose2d(-16, -9, Math.toRadians(0)))
                .build().schedule();
    }

    @Override
    public void loop() {
        //telemetry.addLine(Mercurial.INSTANCE.getActiveCommandSnapshot().toString());
    }
}
;