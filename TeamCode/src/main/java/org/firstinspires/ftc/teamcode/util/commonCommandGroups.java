package org.firstinspires.ftc.teamcode.util;

import org.firstinspires.ftc.teamcode.subsystems.deposit;
import org.firstinspires.ftc.teamcode.subsystems.intake;

import dev.frozenmilk.mercurial.commands.Command;
import dev.frozenmilk.mercurial.commands.groups.Parallel;

public class commonCommandGroups {
    public Command transfer(){
        return new Parallel(deposit.closeClaw(), intake.openClaw());
    }
}
