package org.firstinspires.ftc.teamcode.opmodes;

import static org.firstinspires.ftc.teamcode.subsystems.CommandGroups.intakeAuto;
import static org.firstinspires.ftc.teamcode.subsystems.CommandGroups.liftHigh;
import static org.firstinspires.ftc.teamcode.subsystems.CommandGroups.retract;
import static org.firstinspires.ftc.teamcode.subsystems.CommandGroups.transfer;

import com.acmerobotics.roadrunner.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.subsystems.CommandGroups;
import org.firstinspires.ftc.teamcode.subsystems.Wavedash;
import org.firstinspires.ftc.teamcode.subsystems.arm;
import org.firstinspires.ftc.teamcode.subsystems.deposit;
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

@TeleOp
@Mercurial.Attach
@arm.Attach
@deposit.Attach
@intake.Attach
@lift.Attach
@extendo.Attach
@Wavedash.Attach
@subAuto.Attach
@BulkRead.Attach
@LoopTimes.Attach
@SlothFinder.Attach
@Telem.Attach
public class visionTest extends OpMode {
    private final Pose2d initialPose = new Pose2d(-36, -62.5, Math.toRadians(180));
    private final Pose2d depositSpot = new Pose2d(-53, -55, Math.toRadians(230));

    private Command intakeSample() {
        return new Sequential(CommandGroups.intake(), intake.closeClaw(), new Wait(.15));
    }

    private Command intakeSampleShort() {
        return new Sequential(CommandGroups.intakeAutoShort(), intake.closeClaw(), new Wait(.15));
    }

    private Command transferAndLift() {
        return new Sequential(retract(), transfer(), liftHigh());
    }

    @Override
    public void init() {
        Mercurial.gamepad1().b().onTrue(subAuto.setRed());
        Mercurial.gamepad1().x().onTrue(subAuto.setBlue());
    }
    @Override
    public void start(){
        Wavedash.p2pBuilder(new Pose2d(0,0,Math.toRadians(180)))
                .stopAndAdd(extendo.goTo(465), intake.wristVision())
                .waitSeconds(2)
                .pidRealitive(subAuto.xOffset(), subAuto.yOffset(), () -> 0)
                .stopAndAdd(CommandGroups.intake())
                .build().schedule();
    }

    @Override
    public void loop(){
        telemetry.addData("X", subAuto.xOffset().getAsDouble());
        telemetry.addData("Y", subAuto.yOffset().getAsDouble());
    }
}

