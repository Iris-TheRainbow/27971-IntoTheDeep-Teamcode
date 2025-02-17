package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.subsystems.arm;
import org.firstinspires.ftc.teamcode.subsystems.deposit;

import dev.frozenmilk.mercurial.Mercurial;

@TeleOp
@Mercurial.Attach
@deposit.Attach
@arm.Attach
public class TestingOpMode extends OpMode {
    @Override
    public void init(){
        Mercurial.gamepad1().x().onTrue(arm.armUp());
        Mercurial.gamepad1().y().onTrue(deposit.wristDeposit());
        Mercurial.gamepad1().a().onTrue(deposit.toggleClaw());
    }
    @Override
    public void loop(){

    }
}
