package org.firstinspires.ftc.teamcode.commandbase;

import dev.frozenmilk.mercurial.Mercurial;
import dev.frozenmilk.mercurial.commands.Command;
import dev.frozenmilk.mercurial.commands.Lambda;

public class CommandUtil {
    public static Lambda proxiedCommand(Command command){
        return new Lambda("Proxied " + command.toString())
                .setInit(command::schedule)
                .setFinish(() -> !Mercurial.isScheduled(command));
    }
}
