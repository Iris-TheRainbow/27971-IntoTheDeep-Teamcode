package org.firstinspires.ftc.teamcode.util;

import com.acmerobotics.roadrunner.Pose2d;

import java.util.function.DoubleSupplier;


public class LazyPose2d{
    private final DoubleSupplier x, y, h;
    private Pose2d pose;

    public LazyPose2d(DoubleSupplier x, DoubleSupplier y, DoubleSupplier h){
        this.x = x;
        this.y = y;
        this.h = h;
    }
    public Pose2d value(){
        return this.pose;
    }
    public void evaluate(){
        this.pose = new Pose2d(x.getAsDouble(), y.getAsDouble(), h.getAsDouble());
    }
}
