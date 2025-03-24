package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.commandbase.subsystems.arm;
import org.firstinspires.ftc.teamcode.commandbase.subsystems.deposit;
import org.firstinspires.ftc.teamcode.commandbase.subsystems.extendo;
import org.firstinspires.ftc.teamcode.commandbase.subsystems.lift;
import org.firstinspires.ftc.teamcode.util.features.Telem;

import dev.frozenmilk.mercurial.Mercurial;

@Mercurial.Attach
@lift.Attach
@extendo.Attach
@deposit.Attach
@arm.Attach
@TeleOp
@Telem.Attach
public class resetAll extends OpMode {
    @Override
    public void init() {

    }
    @Override
    public void start(){
        arm.armTransfer().schedule();
        deposit.wristSepc().schedule();
        lift.reset();
        extendo.reset();
    }
    @Override
    public void loop() {

    }
}
