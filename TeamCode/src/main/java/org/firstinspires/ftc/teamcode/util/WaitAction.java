package org.firstinspires.ftc.teamcode.util;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;

import dev.frozenmilk.mercurial.Mercurial;
import dev.frozenmilk.mercurial.commands.Command;

public class WaitAction implements Action {
    private final double seconds;
    private final double start;
    public WaitAction(double seconds) { this.seconds = seconds; start = (double)(System.currentTimeMillis() / 1000); }

    @Override
    public boolean run(@NonNull TelemetryPacket packet) {
        return ((start + seconds) < (double) (System.currentTimeMillis() / 1000));
    }
}