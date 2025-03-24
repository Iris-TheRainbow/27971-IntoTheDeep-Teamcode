package org.firstinspires.ftc.teamcode.pathing;


import com.acmerobotics.roadrunner.Pose2d;

import org.firstinspires.ftc.teamcode.commandbase.commandUtil;

import java.util.ArrayList;
import java.util.List;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

import dev.frozenmilk.mercurial.commands.Command;
import dev.frozenmilk.mercurial.commands.Lambda;
import dev.frozenmilk.mercurial.commands.groups.Parallel;
import dev.frozenmilk.mercurial.commands.groups.Sequential;
import dev.frozenmilk.mercurial.commands.util.Wait;

public class PidToPointBuilder {
    private final List<Command> commands = new ArrayList<>();
    private Pose2d pose;
    private final PidToPointCommand pidCommand;
    private final Supplier<Pose2d> currentPose;
    public PidToPointBuilder(Pose2d pose, PidToPointCommand command, Supplier<Pose2d> currentPose){ this.pose = pose; pidCommand = command; this.currentPose = currentPose; }
    public PidToPointBuilder pidAfter(double seconds, Pose2d pose){ return pidAfter(seconds, pose, 1, 3); }
    public PidToPointBuilder pidAfter(double seconds, Pose2d pose, double translationalAccuracy, double headingAccuracy){
        this.pose = pose;
        commands.add(
                new Sequential(new Wait(seconds),
                        pidCommand.get(
                                () -> this.pose.position.x,
                                () ->  this.pose.position.y,
                                () ->  this.pose.heading.toDouble(),
                                translationalAccuracy,
                                headingAccuracy)));
        return this;
    }
    public PidToPointBuilder pidTo(Pose2d pose){
        return pidTo(pose, 1, 3);
    }

    public PidToPointBuilder pidTo(DoubleSupplier x, DoubleSupplier y, DoubleSupplier h){
        return pidTo(x, y ,h , 1, 3);
    }
    public PidToPointBuilder pidTo(DoubleSupplier x, DoubleSupplier y, DoubleSupplier h, double translationalAccuracy, double headingAccuracy){
        commands.add(pidCommand.get(x, y, h, translationalAccuracy,  headingAccuracy));
        return this;
    }
    public PidToPointBuilder pidTo(Pose2d pose, double translationalAccuracy, double headingAccuracy){
        this.pose = pose;
        double x, y, h;
        commands.add(pidCommand.get(
                () -> pose.position.x,
                () ->  pose.position.y,
                pose.heading::toDouble,
                translationalAccuracy,
                headingAccuracy));
        return this;
    }
    public PidToPointBuilder pidRealitive(DoubleSupplier x, DoubleSupplier y, DoubleSupplier h){
        return pidRealitive(x, y ,h , 1, 3);
    }

    public PidToPointBuilder pidRealitive(DoubleSupplier x, DoubleSupplier y, double h){
        return pidRealitive(x, y ,() -> h , 1, 3);
    }
    public PidToPointBuilder pidRealitive(DoubleSupplier x, DoubleSupplier y, DoubleSupplier h, double translationalAccuracy, double headingAccuracy){
        commands.add(pidCommand.get(
                () -> this.pose.position.x + x.getAsDouble(),
                () ->  this.pose.position.y + y.getAsDouble(),
                () ->  this.pose.heading.toDouble() + h.getAsDouble(),
                translationalAccuracy,
                headingAccuracy));
        return this;
    }
    public PidToPointBuilder afterTime(double time, Command... commandToAfter){
        int index = commands.size() - 1;
        commands.set(
                index,
                new Parallel(commands.get(index),
                        new Sequential(new Wait(time),
                                new Parallel(commandToAfter))));
        return this;
    }
    public PidToPointBuilder afterDisp(double disp, Command... commandToAfter){
        int index = commands.size() - 1;
        commands.set(index, new Parallel(commands.get(index), new Sequential(new Lambda("wait until disp").setFinish(() -> (currentPose.get().minusExp(pose).position.norm() < disp)), new Parallel(commandToAfter))));
        return this;
    }

    public PidToPointBuilder duringLast(Command... command){ return afterTime(0, command); }
    public PidToPointBuilder stopAndAdd(Command... command){ commands.add(new Parallel(command)); return this; }
    public PidToPointBuilder waitSeconds(double seconds){ return stopAndAdd(new Wait(seconds)); }
    public Command build(){ commands.replaceAll(commandUtil::proxiedCommand); return new Sequential(commands); }

    public interface PidToPointCommand{
        public Command get(DoubleSupplier x, DoubleSupplier y, DoubleSupplier h, double translationalAccuracy, double headingAccuracy);
    }
}