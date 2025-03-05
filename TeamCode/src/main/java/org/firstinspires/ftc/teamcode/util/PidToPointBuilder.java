package org.firstinspires.ftc.teamcode.util;


import com.acmerobotics.roadrunner.Pose2d;

import org.firstinspires.ftc.teamcode.subsystems.Wavedash;

import java.util.ArrayList;
import java.util.List;
import java.util.function.DoubleSupplier;

import dev.frozenmilk.mercurial.commands.Command;
import dev.frozenmilk.mercurial.commands.Lambda;
import dev.frozenmilk.mercurial.commands.groups.Parallel;
import dev.frozenmilk.mercurial.commands.groups.Sequential;
import dev.frozenmilk.mercurial.commands.util.Wait;

public class PidToPointBuilder {
    private final List<Command> commands = new ArrayList<>();
    private Pose2d pose;
    public PidToPointBuilder(Pose2d pose){
        this.pose = pose;
    }
    public PidToPointBuilder pidAfter(double seconds, Pose2d pose){ return pidAfter(seconds, pose, 1, 3); }
    public PidToPointBuilder pidAfter(double seconds, Pose2d pose, double translationalAccuracy, double headingAccuracy){
        this.pose = pose;
        commands.add(
                new Sequential(new Wait(seconds),
                        Wavedash.PIDToPoint(
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
        commands.add(Wavedash.PIDToPoint(x, y, h, translationalAccuracy,  headingAccuracy));
        return this;
    }
    public PidToPointBuilder pidTo(Pose2d pose, double translationalAccuracy, double headingAccuracy){
        this.pose = pose;
        double x, y, h;
        commands.add(Wavedash.PIDToPoint(
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
        commands.add(Wavedash.PIDToPoint(
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
        commands.set(index, new Parallel(commands.get(index), new Sequential(new Lambda("wait until disp").setFinish(() -> (Wavedash.getPose().minusExp(pose).position.norm() < disp)), new Parallel(commandToAfter))));
        return this;
    }

    public PidToPointBuilder duringLast(Command... command){ return afterTime(0, command); }
    public PidToPointBuilder stopAndAdd(Command... command){ commands.add(new Parallel(command)); return this; }
    public PidToPointBuilder waitSeconds(double seconds){ return stopAndAdd(new Wait(seconds)); }
    public Command build(){ commands.replaceAll(commandUtil::proxiedCommand); return new Sequential(commands); }
}

