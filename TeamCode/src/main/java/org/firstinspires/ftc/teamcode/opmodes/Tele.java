package org.firstinspires.ftc.teamcode.opmodes;

import static org.firstinspires.ftc.teamcode.commandbase.CommandGroups.depositSpec;
import static org.firstinspires.ftc.teamcode.commandbase.CommandGroups.liftHigh;
import static org.firstinspires.ftc.teamcode.commandbase.CommandGroups.liftMedium;
import static org.firstinspires.ftc.teamcode.commandbase.CommandGroups.prepDepo;
import static org.firstinspires.ftc.teamcode.commandbase.CommandGroups.retract;
import static org.firstinspires.ftc.teamcode.commandbase.CommandGroups.transfer;
import static org.firstinspires.ftc.teamcode.commandbase.commandUtil.proxiedCommand;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.commandbase.CommandGroups;
import org.firstinspires.ftc.teamcode.commandbase.subsystems.DepositArm;
import org.firstinspires.ftc.teamcode.commandbase.subsystems.DepositClaw;
import org.firstinspires.ftc.teamcode.commandbase.subsystems.Extendo;
import org.firstinspires.ftc.teamcode.commandbase.subsystems.IntakeClaw;
import org.firstinspires.ftc.teamcode.commandbase.subsystems.IntakeRotate;
import org.firstinspires.ftc.teamcode.commandbase.subsystems.IntakeWrist;
import org.firstinspires.ftc.teamcode.commandbase.subsystems.Drive;
import org.firstinspires.ftc.teamcode.commandbase.subsystems.Lift;
import org.firstinspires.ftc.teamcode.commandbase.subsystems.DepositWrist;
import org.firstinspires.ftc.teamcode.util.features.BulkRead;
import org.firstinspires.ftc.teamcode.util.features.LoopTimes;
import org.firstinspires.ftc.teamcode.util.features.SlothFinder;
import org.firstinspires.ftc.teamcode.util.features.Telem;


import dev.frozenmilk.mercurial.Mercurial;
import dev.frozenmilk.mercurial.commands.groups.Advancing;
import dev.frozenmilk.mercurial.commands.groups.Parallel;
import dev.frozenmilk.mercurial.commands.groups.Sequential;
import dev.frozenmilk.mercurial.commands.util.IfElse;
import dev.frozenmilk.mercurial.commands.util.Wait;

@TeleOp
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
public class Tele extends OpMode {
    @Override
    public void init() {
        Lift.liftOffset = 0;
        Mercurial.gamepad2().y().onTrue(new Advancing(IntakeWrist.wristTransfer(), DepositWrist.wristTransfer(), DepositArm.armTransfer(), DepositClaw.closeClaw(),new Wait(.3), IntakeClaw.openClaw()));
        //lift to high basket
        Mercurial.gamepad1().y().onTrue(new Sequential(Drive.nerfDrive(1), retract(), transfer(), liftHigh()));
        //lift to high bar
        Mercurial.gamepad1().x().onTrue(new Sequential(Drive.nerfDrive(1),liftMedium()));
        //retract extendo and transfer
        Mercurial.gamepad1().a().onTrue(new Sequential(Drive.nerfDrive(1), retract(), transfer(), prepDepo(), DepositWrist.wristSepc()));
        //extend
        Mercurial.gamepad1().b().onTrue(new Sequential(Drive.nerfDrive(.5), CommandGroups.intake()));
        //toggle for wrist
        Mercurial.gamepad1().leftBumper().onTrue(new Advancing(IntakeWrist.wristTransfer(), IntakeWrist.wristIntake()));
        //toggle for claw
        //Mercurial.gamepad1().rightBumper().onTrue(new Parallel(IntakeClaw.toggleClaw(), DepositClaw.toggleClaw()));
        Mercurial.gamepad1().rightBumper().onTrue(new IfElse(() -> Lift.getTarget() > 1000, proxiedCommand(new Sequential(DepositClaw.openClaw(), new Wait(.05), new Parallel(DepositArm.armExtend(), new Sequential(new Wait(.1), retract())))), proxiedCommand(new Parallel(IntakeClaw.toggleClaw(), DepositClaw.toggleClaw()))));
        //retract then depo for spec
        Mercurial.gamepad1().dpadLeft().onTrue(depositSpec());
        //hang extend
        Mercurial.gamepad1().dpadUp().onTrue(Lift.offset(10));
        //hang retract
        Mercurial.gamepad1().dpadDown().onTrue(Lift.offset(-10));

        Mercurial.gamepad1().back().whileTrue(proxiedCommand(Lift.retract()));
        gamepad1.setLedColor(255, 0, 0, 999999999);
    }

    @Override
    public void loop() {
        telemetry.addLine(Mercurial.INSTANCE.getActiveCommandSnapshot().toString());
    }
}
