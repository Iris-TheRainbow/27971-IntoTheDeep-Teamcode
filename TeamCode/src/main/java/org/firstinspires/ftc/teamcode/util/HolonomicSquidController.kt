package org.firstinspires.ftc.teamcode.util

import com.acmerobotics.roadrunner.Pose2d
import com.acmerobotics.roadrunner.Pose2dDual
import com.acmerobotics.roadrunner.PoseVelocity2d
import com.acmerobotics.roadrunner.PoseVelocity2dDual
import com.acmerobotics.roadrunner.Time
import com.acmerobotics.roadrunner.Vector2d

class HolonomicSquidController(
    @JvmField
    val axialPosGain: Double,
    @JvmField
    val lateralPosGain: Double,
    @JvmField
    val headingGain: Double,
    @JvmField
    val axialVelGain: Double,
    @JvmField
    val lateralVelGain: Double,
    @JvmField
    val headingVelGain: Double,
) {
    constructor(
        axialPosGain: Double,
        lateralPosGain: Double,
        headingGain: Double,
    ) : this(axialPosGain, lateralPosGain, headingGain, 0.0, 0.0, 0.0)

    /**
     * Computes the velocity and acceleration command. The frame `Target` is the reference robot, and the frame `Actual`
     * is the measured, physical robot.
     *
     * @return velocity command in the actual frame
     */
    fun compute(
        targetPose: Pose2dDual<Time>,
        actualPose: Pose2d,
        actualVelActual: PoseVelocity2d,
    ): PoseVelocity2dDual<Time> {
        // TODO: Are these names useful for anyone else?
        val targetVelWorld = targetPose.velocity()
        val txTargetWorld = Pose2dDual.constant<Time>(targetPose.value().inverse(), 2)
        val targetVelTarget = txTargetWorld * targetVelWorld

        val velErrorActual = targetVelTarget.value() - actualVelActual

        val error = targetPose.value().minusExp(actualPose)
        return targetVelTarget +
                PoseVelocity2d(
                    Vector2d(
                        axialPosGain * signedFunc(Math::sqrt, error.position.x),
                        lateralPosGain * signedFunc(Math::sqrt, error.position.y),
                    ),
                    headingGain * signedFunc(Math::sqrt, error.heading.log()),
                ) +
                PoseVelocity2d(
                    Vector2d(
                        axialVelGain * velErrorActual.linearVel.x,
                        lateralVelGain * velErrorActual.linearVel.y,
                    ),
                    headingVelGain * velErrorActual.angVel,
                )
    }
}