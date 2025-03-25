package org.firstinspires.ftc.teamcode.opmodes.autos;

import static org.firstinspires.ftc.teamcode.commandbase.CommandGroups.depositSpec;
import static org.firstinspires.ftc.teamcode.commandbase.CommandGroups.liftMedium;
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
import org.firstinspires.ftc.teamcode.util.features.BulkRead;
import org.firstinspires.ftc.teamcode.util.features.LoopTimes;
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
public class SpecimineAuto extends OpMode {
    private final Pose2d initialPose = new Pose2d(12, -62.5, Math.toRadians(90));
    private Command driveCommand;
    private Command intakeSample(){
        return new Sequential(CommandGroups.intake(), new Wait(.1), IntakeClaw.closeClaw(), new Wait(.2));
    }
    private Command intakeSampleShort(){
        return new Sequential(CommandGroups.intakeAutoShort(), new Wait(.25), IntakeClaw.closeClaw(), new Wait(.45));
    }
    private Command transferAndLift(){
        return new Sequential(retract(), transfer(), liftMedium());
    }
    public Pose2d specDepo = new Pose2d(0, -34, Math.toRadians(90));
    public Pose2d slideSpec = new Pose2d(-2, -34, Math.toRadians(90));
    public Pose2d prepDepo = new Pose2d(0, -40, Math.toRadians(90));
    public Pose2d specIntake =  new Pose2d(19.5,-58, Math.toRadians(180));

    @Override
    public void init() {
        Drive.p2p(initialPose)
                .stopAndAdd(DepositClaw.closeClaw())
                .waitSeconds(.5)
                .pidTo(new Pose2d(-4, -32, Math.toRadians(90)))
                .duringLast(liftMedium(), DepositClaw.closeClaw(), Extendo.goTo(0))
                .stopAndAdd(depositSpec())
                //strafe left
                .pidTo(new Pose2d(33, -40, Math.toRadians(0)), 2, 9999)
                .duringLast(retract())
                .pidTo(new Pose2d(33, -18, Math.toRadians(270)), 8, 9999)
                .pidTo(new Pose2d(44, -18, Math.toRadians(270)), 6, 9999)
                //first push
                .pidTo(new Pose2d(46, -52, Math.toRadians(270)), 4, 9999)
                .pidTo(new Pose2d(46, -14, Math.toRadians(270)), 8, 9999)
                .pidTo(new Pose2d(53, -14, Math.toRadians(270)), 6, 9999)
                //second push
                .pidTo(new Pose2d(52, -52, Math.toRadians(270)), 4, 9999)

                .repeat(5, -1)
                .pidTo(specIntake)
                .waitSeconds(1.0)
                .stopAndAdd(intakeSample())
                .pidTo(prepDepo)
                .duringLast(transferAndLift())
                .pidTo(new Pose2d(0, -34.5, Math.toRadians(90)))
                .stopAndAdd(depositSpec())
                .stopRepeat()
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
