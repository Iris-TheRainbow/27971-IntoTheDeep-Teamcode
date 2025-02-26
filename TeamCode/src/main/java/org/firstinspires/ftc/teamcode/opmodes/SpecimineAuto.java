package org.firstinspires.ftc.teamcode.opmodes;

import static org.firstinspires.ftc.teamcode.subsystems.CommandGroups.depositSpec;
import static org.firstinspires.ftc.teamcode.subsystems.CommandGroups.intakeAuto;
import static org.firstinspires.ftc.teamcode.subsystems.CommandGroups.liftHigh;
import static org.firstinspires.ftc.teamcode.subsystems.CommandGroups.liftMedium;
import static org.firstinspires.ftc.teamcode.subsystems.CommandGroups.retract;
import static org.firstinspires.ftc.teamcode.subsystems.CommandGroups.transfer;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.subsystems.CommandGroups;
import org.firstinspires.ftc.teamcode.subsystems.Wavedash;
import org.firstinspires.ftc.teamcode.subsystems.arm;
import org.firstinspires.ftc.teamcode.subsystems.deposit;
import org.firstinspires.ftc.teamcode.subsystems.extendo;
import org.firstinspires.ftc.teamcode.subsystems.intake;
import org.firstinspires.ftc.teamcode.subsystems.lift;
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
@Wavedash.Attach
@BulkRead.Attach
@LoopTimes.Attach
@SlothFinder.Attach
@Telem.Attach
public class SpecimineAuto extends OpMode {
    private final Pose2d initialPose = new Pose2d(12, -62.5, Math.toRadians(90));
    private Command driveCommand;
    private Command intakeSample(){
        return new Sequential(CommandGroups.intake(), new Wait(.1), intake.closeClaw(), new Wait(.2));
    }
    private Command intakeSampleShort(){
        return new Sequential(CommandGroups.intakeAutoShort(), new Wait(.25), intake.closeClaw(), new Wait(.45));
    }
    private Command transferAndLift(){
        return new Sequential(retract(), transfer(), liftMedium());
    }
    public Pose2d specDepo = new Pose2d(0, -34, Math.toRadians(90));
    public Pose2d slideSpec = new Pose2d(-2, -34, Math.toRadians(90));
    public Pose2d prepDepo = new Pose2d(0, -40, Math.toRadians(90));
    public Pose2d specIntkae =  new Pose2d(19.5,-58, Math.toRadians(180));

    @Override
    public void init() {
        driveCommand = Wavedash.p2pBuilder(initialPose)
                .pidTo(new Pose2d(0-4, -34, Math.toRadians(90)))
                .duringLast(liftMedium(), deposit.closeClaw(), extendo.goTo(0))
                .stopAndAdd(depositSpec())
                //strafe left
                .pidTo(new Pose2d(33, -40, Math.toRadians(0)), 2, 9999)
                .duringLast(retract())
                .pidTo(new Pose2d(33, -18, Math.toRadians(270)), 8, 9999)
                .pidTo(new Pose2d(44, -18, Math.toRadians(270)), 6, 9999)
                //first push
                .pidTo(new Pose2d(46, -52, Math.toRadians(270)), 4, 9999)
                .pidTo(new Pose2d(46, -14, Math.toRadians(270)), 8, 9999)
                .pidTo(new Pose2d(54, -14, Math.toRadians(270)), 6, 9999)
                //second push
                .pidTo(new Pose2d(52, -52, Math.toRadians(270)), 4, 9999)
                //scoring
                .pidTo(specIntkae)
                .stopAndAdd(intakeSample())
                .pidTo(prepDepo)
                .duringLast(transferAndLift())
                .pidTo(new Pose2d(0-3, -34, Math.toRadians(90)))
                .stopAndAdd(depositSpec())
                .pidTo(specIntkae)
                .stopAndAdd(intakeSample())
                .pidTo(prepDepo)
                .duringLast(transferAndLift())
                .pidTo(specDepo)
                .pidTo(new Pose2d(0-2, -34, Math.toRadians(90)))
                .stopAndAdd(depositSpec())
                .pidTo(specIntkae)
                .stopAndAdd(intakeSample())
                .pidTo(prepDepo)
                .duringLast(transferAndLift())
                .pidTo(new Pose2d(0-1, -34., Math.toRadians(90)))
                .stopAndAdd(depositSpec())
                .pidTo(new Pose2d(24,-54, Math.toRadians(160)))
                .duringLast(extendo.goTo(435))
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