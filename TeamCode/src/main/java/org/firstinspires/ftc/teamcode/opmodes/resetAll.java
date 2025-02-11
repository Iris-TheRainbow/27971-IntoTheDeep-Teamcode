package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.subsystems.extendo;
import org.firstinspires.ftc.teamcode.subsystems.lift;

import dev.frozenmilk.mercurial.Mercurial;

@Mercurial.Attach
@lift.Attach
@extendo.Attach
@TeleOp
public class resetAll extends OpMode {
    @Override
    public void init() {

    }
    @Override
    public void start(){
        lift.reset();
        extendo.reset();
    }
    @Override
    public void loop() {

    }
}
