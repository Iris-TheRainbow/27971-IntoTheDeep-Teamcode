package org.firstinspires.ftc.teamcode.opmodes;

import com.outoftheboxrobotics.photoncore.Photon;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.subsystems.arm;
import org.firstinspires.ftc.teamcode.subsystems.claw;
import org.firstinspires.ftc.teamcode.subsystems.drive;
import org.firstinspires.ftc.teamcode.subsystems.lift;
import org.firstinspires.ftc.teamcode.subsystems.wrist;
import org.firstinspires.ftc.teamcode.util.LoopTimes;


import dev.frozenmilk.dairy.core.util.features.BulkRead;
import dev.frozenmilk.mercurial.Mercurial;
import dev.frozenmilk.mercurial.commands.groups.Advancing;
import dev.frozenmilk.mercurial.commands.groups.Parallel;

@com.qualcomm.robotcore.eventloop.opmode.TeleOp(name = "TeleOp")
@Mercurial.Attach
@drive.Attach
@arm.Attach
@wrist.Attach
@claw.Attach
@lift.Attach
@BulkRead.Attach
@LoopTimes.Attach
public class TeleOp extends OpMode {
    @Override
    public void init() {
        Mercurial.gamepad1().start().onTrue(drive.zeroHeading());
        Mercurial.gamepad1().a().onTrue(new Parallel(lift.goTo(0), arm.armOut(), wrist.wristFlat(), claw.openClaw()));
        Mercurial.gamepad1().b().onTrue(new Parallel(lift.goTo(1000), arm.armUp(), wrist.wristFlat(), claw.closeClaw()));
        Mercurial.gamepad1().x().onTrue(new Parallel(lift.goTo(3400), arm.armUp(), wrist.wristFlat(), claw.closeClaw()));
        Mercurial.gamepad1().y().onTrue(new Parallel(lift.goTo(5000), arm.armUp(), wrist.wristFlat(), claw.closeClaw()));
        Mercurial.gamepad1().rightBumper().onTrue(new Advancing(claw.closeClaw(), claw.openClaw()));
        Mercurial.gamepad1().leftBumper().onTrue(new Advancing(wrist.wristDown(), wrist.wristFlat()));
        Mercurial.gamepad1().dpadUp().onTrue(new Parallel(lift.goTo(5000), wrist.wristDown(), arm.armStow(), claw.closeClaw()));
        Mercurial.gamepad1().dpadDown().onTrue(new Parallel(lift.goTo(-15), wrist.wristDown(), arm.armStow(), claw.closeClaw()));
        Mercurial.gamepad1().dpadRight().onTrue(new Parallel(arm.armStow(), wrist.wristFlat(), claw.closeClaw()));
    }

    @Override
    public void loop() {

    }
}
