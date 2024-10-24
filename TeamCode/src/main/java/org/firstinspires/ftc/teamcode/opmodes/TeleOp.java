package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.subsystems.drive;

import java.util.NavigableMap;

import dev.frozenmilk.dairy.core.util.features.BulkRead;
import dev.frozenmilk.mercurial.Mercurial;

@com.qualcomm.robotcore.eventloop.opmode.TeleOp(name = "TeleOp")
@Mercurial.Attach
@drive.Attach
@BulkRead.Attach
public class TeleOp extends OpMode {
    @Override
    public void init() {
        Mercurial.gamepad1().start().onTrue(drive.zeroHeading());
    }

    @Override
    public void loop() {

    }
}
