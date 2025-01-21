package org.firstinspires.ftc.teamcode.util;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;

import dev.frozenmilk.mercurial.Mercurial;
import dev.frozenmilk.mercurial.commands.Command;

public class MercurialAction implements Action {
    private final Command command;
    private boolean initialized = false;
    public MercurialAction(Command command) { this.command = command; }

    @Override
    public boolean run(@NonNull TelemetryPacket packet) {
        if (!initialized) {
            command.schedule();
            initialized = true;
        }
        packet.addLine("Scheduled " + command.toString());

        return Mercurial.isScheduled(command);
    }
}
