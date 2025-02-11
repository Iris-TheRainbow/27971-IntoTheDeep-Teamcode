package org.firstinspires.ftc.teamcode.util;

import dev.frozenmilk.mercurial.commands.Command;
import dev.frozenmilk.mercurial.commands.Lambda;

public class commandUtil {
    public static Lambda instantifiedCommand(Command command){
        return new Lambda("instantified" + command.toString())
                .setInit(command::schedule)
                .setFinish(() -> true);
    }
}
