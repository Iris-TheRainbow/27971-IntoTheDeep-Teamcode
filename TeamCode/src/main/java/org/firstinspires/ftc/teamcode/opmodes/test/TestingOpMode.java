package org.firstinspires.ftc.teamcode.opmodes.test;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.commandbase.subsystems.DepositArm;
import org.firstinspires.ftc.teamcode.commandbase.subsystems.DepositClaw;
import org.firstinspires.ftc.teamcode.commandbase.subsystems.DepositWrist;
import org.firstinspires.ftc.teamcode.util.features.Telem;

import dev.frozenmilk.mercurial.Mercurial;

@TeleOp
@Mercurial.Attach
@DepositWrist.Attach
@DepositArm.Attach
@Telem.Attach
public class TestingOpMode extends OpMode {
    @Override
    public void init(){
        Mercurial.gamepad1().x().onTrue(DepositArm.armUp());
        Mercurial.gamepad1().y().onTrue(DepositWrist.wristDeposit());
        Mercurial.gamepad1().a().onTrue(DepositClaw.toggleClaw());
    }
    @Override
    public void loop(){
    }
}
