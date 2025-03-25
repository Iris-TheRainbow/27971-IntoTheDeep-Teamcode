package org.firstinspires.ftc.teamcode.opmodes.autos;

import static org.firstinspires.ftc.teamcode.commandbase.CommandGroups.intakeAuto;
import static org.firstinspires.ftc.teamcode.commandbase.CommandGroups.liftHigh;
import static org.firstinspires.ftc.teamcode.commandbase.CommandGroups.retract;
import static org.firstinspires.ftc.teamcode.commandbase.CommandGroups.transfer;

import com.acmerobotics.roadrunner.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.commandbase.CommandGroups;
import org.firstinspires.ftc.teamcode.commandbase.subsystems.DepositArm;
import org.firstinspires.ftc.teamcode.commandbase.subsystems.DepositClaw;
import org.firstinspires.ftc.teamcode.commandbase.subsystems.DepositWrist;
import org.firstinspires.ftc.teamcode.commandbase.subsystems.Drive;
import org.firstinspires.ftc.teamcode.commandbase.subsystems.Extendo;
import org.firstinspires.ftc.teamcode.commandbase.subsystems.IntakeClaw;
import org.firstinspires.ftc.teamcode.commandbase.subsystems.IntakeRotate;
import org.firstinspires.ftc.teamcode.commandbase.subsystems.IntakeWrist;
import org.firstinspires.ftc.teamcode.commandbase.subsystems.Lift;
import org.firstinspires.ftc.teamcode.util.features.LoopTimes;

import org.firstinspires.ftc.teamcode.util.features.BulkRead;
import org.firstinspires.ftc.teamcode.util.features.SlothFinder;
import org.firstinspires.ftc.teamcode.util.features.Telem;

import dev.frozenmilk.mercurial.Mercurial;
import dev.frozenmilk.mercurial.commands.Command;
import dev.frozenmilk.mercurial.commands.groups.Sequential;
import dev.frozenmilk.mercurial.commands.util.Wait;

@Autonomous
@Mercurial.Attach
@DepositArm.Attach
@DepositWrist.Attach
@DepositClaw.Attach
@IntakeWrist.Attach
@IntakeClaw.Attach
@IntakeRotate.Attach
@Lift.Attach
@Extendo.Attach
@Drive.Attach
@BulkRead.Attach
@LoopTimes.Attach
@SlothFinder.Attach
@Telem.Attach
public class AdvancedSampleAuto extends OpMode {
    private final Pose2d initialPose = new Pose2d(-36, -62.5, Math.toRadians(180));
    private final Pose2d depositSpot = new Pose2d(-53, -55, Math.toRadians(230));
    private Command intakeSample(){
        return new Sequential(CommandGroups.intake(), IntakeClaw.closeClaw(), new Wait(.15));
    }
    private Command intakeSampleShort(){
        return new Sequential(CommandGroups.intakeAutoShort(), IntakeClaw.closeClaw(), new Wait(.15));
    }
    private Command transferAndLift(){
        return new Sequential(retract(), transfer(), liftHigh());
    }

    @Override
    public void init() {
    }
    @Override
    public void start(){
        Drive.p2p(initialPose)
                .pidTo(depositSpot)
                .duringLast(liftHigh())

                //cycle1
                .pidTo(new Pose2d(-45.50, -54, Math.toRadians(90-180)))
                .duringLast(DepositClaw.openClaw())
                .stopAndAdd(intakeSample())
                .pidTo(depositSpot)
                .duringLast(transferAndLift())
                .stopAndAdd(DepositClaw.openClaw())

                //cycle2
                .pidTo(new Pose2d(-55, -50, Math.toRadians(90-180)), 3, 99999)
                .pidTo(new Pose2d(-55, -53, Math.toRadians(90-180)), .5, 2)
                .duringLast(Extendo.goTo(465))
                .afterTime(.2, Lift.goTo(0))
                .stopAndAdd(intakeSampleShort())
                .pidTo(depositSpot)
                .duringLast(transferAndLift())

                //cycle3
                .pidTo(new Pose2d(-54, -53, Math.toRadians(230)))
                .duringLast(DepositClaw.openClaw())
                .pidTo(new Pose2d(-42, -38, Math.toRadians(-25)))
                .afterTime(.25, Extendo.goTo(435), Lift.goTo(0))
                .duringLast(IntakeRotate.wristRotate(.7))
                .stopAndAdd(new Sequential(intakeAuto(), IntakeRotate.wristRotate(.7), new Wait(.2), IntakeClaw.closeClaw(), new Wait(.15), IntakeRotate.wristRotate(.5)))
                .pidTo(depositSpot)
                .duringLast(transferAndLift())

                //cycle4
                .pidTo(new Pose2d(-44, -13, Math.toRadians(180)), 20, 999) //waypoint so i dont hit the leg
                .duringLast(DepositClaw.openClaw())
                .pidTo(new Pose2d(-30, -9, Math.toRadians(180)))
                .duringLast(new Sequential(new Wait(.5), Extendo.goTo(465)))
                .stopAndAdd(intakeSample())
                .stopAndAdd(IntakeWrist.wristTransfer())
                .pidTo(new Pose2d(-44, -15, Math.toRadians(180)), 20, 999) //waypoint again
                .duringLast(Extendo.goTo(0))
                .pidTo(depositSpot)
                .duringLast(transferAndLift())
                .stopAndAdd(DepositClaw.openClaw())

                //cycle5
                .pidTo(new Pose2d(-44, -13, Math.toRadians(180)), 20, 999) //waypoint so i dont hit the leg
                .pidTo(new Pose2d(-34, -9, Math.toRadians(180)))
                .duringLast(Extendo.goTo(465))
                .stopAndAdd(intakeSample())
                .stopAndAdd(IntakeWrist.wristTransfer())
                .pidTo(new Pose2d(-44, -15, Math.toRadians(180)), 20, 999) //waypoint again
                .duringLast(Extendo.goTo(0))
                .pidTo(depositSpot)
                .duringLast(transferAndLift())
                .stopAndAdd(DepositClaw.openClaw())

                //park
                .pidTo(new Pose2d(-44, -13, Math.toRadians(180)), 20, 999) //waypoint so i dont hit the leg
                .duringLast(new Sequential(new Wait(.1), Lift.goTo(100)))
                .pidTo(new Pose2d(-16, -9, Math.toRadians(0)))
                .build().schedule();
    }

    @Override
    public void loop() {
        //telemetry.addLine(Mercurial.INSTANCE.getActiveCommandSnapshot().toString());
    }
}
