package org.firstinspires.ftc.teamcode.util;

import dev.frozenmilk.mercurial.Mercurial;
import dev.frozenmilk.mercurial.commands.Command;
import dev.frozenmilk.mercurial.commands.Lambda;

public class commandUtil {
    public static Lambda instantifiedCommand(Command command){
        return new Lambda("instantified" + command.toString())
                .setInit(command::schedule)
                .setFinish(() -> true);
    }
    public static Lambda proxiedCommand(Command command){
        return new Lambda("Proxied " + command.toString())
                .setInit(command::schedule)
                .setFinish(() -> !Mercurial.isScheduled(command));
    }
}
