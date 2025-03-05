package org.firstinspires.ftc.teamcode.util;

import androidx.annotation.NonNull;

import com.acmerobotics.roadrunner.Action;

import org.firstinspires.ftc.teamcode.subsystems.Wavedash;

import java.util.Arrays;
import java.util.Collections;
import java.util.HashSet;
import java.util.Set;
import java.util.function.BooleanSupplier;

import dev.frozenmilk.dairy.core.wrapper.Wrapper;
import dev.frozenmilk.mercurial.Mercurial;
import dev.frozenmilk.mercurial.commands.Command;
import dev.frozenmilk.mercurial.commands.Lambda;

public class commandUtil {
    public static Lambda proxiedCommand(Command command){
        return new Lambda("Proxied " + command.toString())
                .setInit(command::schedule)
                .setFinish(() -> !Mercurial.isScheduled(command));
    }
    public static class ActionCmd implements Command{
        private static final Set<Object> requirements = Collections.singleton(Wavedash.INSTANCE);
        private static final Set<Wrapper.OpModeState> runStates = new HashSet<>(Arrays.asList(Wrapper.OpModeState.INIT, Wrapper.OpModeState.ACTIVE));
        private Action act;
        private boolean finished;
        public ActionCmd(Action act){
            this.act = act;
        }

        @Override
        public void initialise() {}

        @Override
        public void execute() {
            finished = !act.run(Telem.telemPacket);
        }

        @Override
        public void end(boolean b) {}

        @Override
        public boolean finished() {
            return finished;
        }

        @NonNull
        @Override
        public Set<Object> getRequirements() {
            return requirements;
        }

        @NonNull
        @Override
        public Set<Wrapper.OpModeState> getRunStates() {
            return runStates;
        }
    }
    public static Lambda ifElse(BooleanSupplier bool, Command iff, Command elsee){
        Command chosen;
        if (bool.getAsBoolean()){
            chosen = iff;
        } else {
            chosen = elsee;
        }
        return new Lambda("ifElse:" + chosen.toString())
                .setInit(chosen::schedule);
    }
}
