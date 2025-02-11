package org.firstinspires.ftc.teamcode.util;

import static org.firstinspires.ftc.teamcode.util.commandUtil.instantifiedCommand;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Pose2dDual;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;

import org.firstinspires.ftc.teamcode.subsystems.Wavedash;

import java.util.ArrayList;
import java.util.List;

import dev.frozenmilk.mercurial.commands.Command;
import dev.frozenmilk.mercurial.commands.Lambda;
import dev.frozenmilk.mercurial.commands.groups.Parallel;
import dev.frozenmilk.mercurial.commands.groups.Sequential;
import dev.frozenmilk.mercurial.commands.util.Wait;

public class PidToPointBuilder {
    private final List<Command> commands = new ArrayList<>();
    private Pose2d pose;
    public PidToPointBuilder pidTo(Pose2d pose){
        this.pose = pose;
        return pidTo(pose, 1, 3);
    }
    public PidToPointBuilder pidTo(Pose2d pose, double translationalAccuracy, double headingAccuracy){
        this.pose = pose;
        commands.add(Wavedash.PIDToPoint(pose, translationalAccuracy, headingAccuracy));
        return this;
    }

    public PidToPointBuilder afterTime(double time, Command commandToAfter){
        int index = commands.size() - 1;
        commands.set(index, new Parallel(commands.get(index), new Sequential(new Wait(time), commandToAfter)));
        return this;
    }
    public PidToPointBuilder duringLast(Command command){
        return afterTime(0, command);

    }
    public PidToPointBuilder stopAndAdd(Command command){
        commands.add(command);
        return this;
    }
    public PidToPointBuilder wait(double seconds){
        return stopAndAdd(new Wait(seconds));
    }
    public Command build(){
        return new Sequential(commands);
    }
}
