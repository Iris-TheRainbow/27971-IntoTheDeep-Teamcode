package org.firstinspires.ftc.teamcode.util.controllers;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.Twist2d;
import com.acmerobotics.roadrunner.Vector2d;

import org.firstinspires.ftc.teamcode.opmodes.Tele;

import java.util.Currency;

public class HolonomicRobotCentricController {
    private final double axialGain,  lateralGain,  headingGain,  axialVelGain,  lateralVelGain,  headingVelGain;
    private double lastX = 0;
    private double lastY = 0;
    public HolonomicRobotCentricController(double axialGain, double lateralGain, double headingGain, double axialVelGain, double lateralVelGain, double headingVelGain){
        this.axialGain = axialGain;
        this.lateralGain = lateralGain;
        this.headingGain = headingGain;
        this.axialVelGain = axialVelGain;
        this.lateralVelGain = lateralVelGain;
        this.headingVelGain = headingVelGain;
    }
    public PoseVelocity2d compute(Pose2d target, Pose2d current){
        Pose2d error = target.minusExp(current);
        double y = -axialGain * ((target.position.x - current.position.x)*Math.cos(current.heading.toDouble()) - (target.position.y - current.position.y)*Math.sin(current.heading.toDouble()));
        double x = lateralGain *((target.position.x - current.position.x)*Math.sin(current.heading.toDouble()) + (target.position.y - current.position.y)*Math.cos(current.heading.toDouble()));
        double h = headingGain * error.heading.toDouble();
        double hypot = Math.hypot(x, y);
        double max = Math.max(1, h);
        return new PoseVelocity2d(
                new Vector2d(
                        x / hypot,
                        y / hypot),
                h / max);
    }
}
