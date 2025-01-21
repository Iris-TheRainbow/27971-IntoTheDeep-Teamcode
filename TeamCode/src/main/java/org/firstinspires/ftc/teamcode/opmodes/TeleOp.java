package org.firstinspires.ftc.teamcode.opmodes;

import static org.firstinspires.ftc.teamcode.subsystems.CommandGroups.depositSpec;
import static org.firstinspires.ftc.teamcode.subsystems.CommandGroups.liftHang;
import static org.firstinspires.ftc.teamcode.subsystems.CommandGroups.liftHigh;
import static org.firstinspires.ftc.teamcode.subsystems.CommandGroups.liftMedium;
import static org.firstinspires.ftc.teamcode.subsystems.CommandGroups.prepDepo;
import static org.firstinspires.ftc.teamcode.subsystems.CommandGroups.retract;
import static org.firstinspires.ftc.teamcode.subsystems.CommandGroups.transfer;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.subsystems.CommandGroups;
import org.firstinspires.ftc.teamcode.subsystems.arm;
import org.firstinspires.ftc.teamcode.subsystems.extendo;
import org.firstinspires.ftc.teamcode.subsystems.intake;
import org.firstinspires.ftc.teamcode.subsystems.drive;
import org.firstinspires.ftc.teamcode.subsystems.lift;
import org.firstinspires.ftc.teamcode.subsystems.deposit;
import org.firstinspires.ftc.teamcode.util.BulkRead;
import org.firstinspires.ftc.teamcode.util.LoopTimes;
import org.firstinspires.ftc.teamcode.util.SlothFinder;


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
@SlothFinder.Attach
public class TeleOp extends OpMode {
    @Override
    public void init() {
        //lift to high basket
        Mercurial.gamepad1().y().onTrue(new Sequential(drive.nerfDrive(1), retract(), transfer(), liftHigh()));
        //lift to high bar
        Mercurial.gamepad1().x().onTrue(new Sequential(drive.nerfDrive(1),liftMedium()));
        //retract extendo and transfer
        Mercurial.gamepad1().a().onTrue(new Sequential(drive.nerfDrive(1), retract(), transfer(), prepDepo(), deposit.wristSepc()));
        //extend
        Mercurial.gamepad1().b().onTrue(new Sequential(drive.nerfDrive(.5), CommandGroups.intake()));
        //toggle for wrist
        Mercurial.gamepad1().leftBumper().onTrue(new Advancing(intake.wristTransfer(), intake.wristIntake()));
        //toggle for claw
        Mercurial.gamepad1().rightBumper().onTrue(new Parallel(intake.toggleClaw(), deposit.toggleClaw()));
        //retract then depo for spec
        Mercurial.gamepad1().dpadLeft().onTrue(depositSpec());
        //hang extend
        Mercurial.gamepad1().dpadUp().onTrue(new Parallel(liftHang()));
        //hang retract
        Mercurial.gamepad1().dpadDown().onTrue(lift.goTo(0));
    }

    @Override
    public void loop() {
    }
}
