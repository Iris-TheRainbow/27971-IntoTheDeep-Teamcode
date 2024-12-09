package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.subsystems.arm;
import org.firstinspires.ftc.teamcode.subsystems.extendo;
import org.firstinspires.ftc.teamcode.subsystems.intake;
import org.firstinspires.ftc.teamcode.subsystems.drive;
import org.firstinspires.ftc.teamcode.subsystems.lift;
import org.firstinspires.ftc.teamcode.subsystems.deposit;
import org.firstinspires.ftc.teamcode.util.LoopTimes;


import dev.frozenmilk.dairy.core.util.features.BulkRead;
import dev.frozenmilk.mercurial.Mercurial;
import dev.frozenmilk.mercurial.commands.groups.Advancing;
import dev.frozenmilk.mercurial.commands.groups.Parallel;
import dev.frozenmilk.mercurial.commands.groups.Sequential;

@com.qualcomm.robotcore.eventloop.opmode.TeleOp(name = "TeleOp")
@Mercurial.Attach
@drive.Attach
@arm.Attach
@deposit.Attach
@intake.Attach
@lift.Attach
@extendo.Attach
@BulkRead.Attach
@LoopTimes.Attach
public class TeleOp extends OpMode {
    @Override
    public void init() {
        //lift
        Mercurial.gamepad1().y().onTrue(new Sequential(deposit.closeClaw(), intake.openClaw(), new Parallel(lift.goTo(3500), deposit.wristDeposit(), arm.armUp())));
        //retract extendo and transfer
        Mercurial.gamepad1().a().onTrue(new Sequential(new Parallel(extendo.goTo(0), lift.goTo(0), intake.wristTransfer(), deposit.wristTransfer(), deposit.openClaw(), arm.armWait()), arm.armTransfer(), deposit.closeClaw(), intake.openClaw(), new Parallel(arm.armUp(), deposit.wristDeposit())));
        //extend
        Mercurial.gamepad1().b().onTrue(new Sequential(new Parallel(extendo.goTo(500), lift.goTo(0), arm.armTransfer(), deposit.wristDeposit(), intake.openClaw()), intake.wristIntake()));
        //toggle for wrist
        Mercurial.gamepad1().leftBumper().onTrue(new Advancing(intake.wristTransfer(), intake.wristIntake()));
        //toggle for claw
        Mercurial.gamepad1().rightBumper().onTrue(new Parallel(intake.toggleClaw(), deposit.toggleClaw()));
        //hang extend
        Mercurial.gamepad1().dpadUp().onTrue(new Parallel(extendo.goTo(0), lift.goTo(3500), intake.wristTransfer(), deposit.wristTransfer(), arm.armTransfer()));
        //hang retract
        Mercurial.gamepad1().dpadDown().onTrue(lift.goTo(0));
    }

    @Override
    public void loop() {
        telemetry.addData("Lift Pos", lift.getLiftPosition());
        telemetry.addData("lift power", lift.getPower());
        telemetry.addData("Extendo Pos", extendo.getLiftPosition());
        telemetry.addData("extendo power", extendo.getPower());
    }
}
