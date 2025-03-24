package org.firstinspires.ftc.teamcode.util.controllers;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.Twist2d;
import com.acmerobotics.roadrunner.Vector2d;

public class HolonomicRobotCentricController {
    private final double axialGain,  lateralGain,  headingGain,  axialVelGain,  lateralVelGain,  headingVelGain;
    public HolonomicRobotCentricController(double axialGain, double lateralGain, double headingGain, double axialVelGain, double lateralVelGain, double headingVelGain){
        this.axialGain = axialGain;
        this.lateralGain = lateralGain;
        this.headingGain = headingGain;
        this.axialVelGain = axialVelGain;
        this.lateralVelGain = lateralVelGain;
        this.headingVelGain = headingVelGain;
    }
    public PoseVelocity2d compute(Pose2d target, Pose2d current){
        Twist2d error = target.minus(current);
        //mayb?
        Vector2d rcVec = current.heading.times(error.line);

        return new PoseVelocity2d(
                new Vector2d(
                        axialGain * rcVec.x,
                        lateralGain * rcVec.y),
                headingGain * error.angle);
    }
}
