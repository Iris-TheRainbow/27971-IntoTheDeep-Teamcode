package org.firstinspires.ftc.teamcode.pathing

import com.acmerobotics.roadrunner.PoseVelocity2d
import com.acmerobotics.roadrunner.TimeTurn
import com.acmerobotics.roadrunner.Trajectory
import com.acmerobotics.roadrunner.Vector2d
import dev.frozenmilk.dairy.core.wrapper.Wrapper
import dev.frozenmilk.mercurial.commands.Command

@JvmDefaultWithoutCompatibility
interface Drive {
    val localizer: Localizer

    fun setDrivePowers(powers: PoseVelocity2d)
    fun setDrivePowersWithFF(powers: PoseVelocity2d)

    fun followTrajectory(trajectory: Trajectory, t: Double): Boolean
    fun turn(turn: TimeTurn, t: Double): Boolean

    fun followTrajectoryCommand(trajectory: Trajectory): Command {
        val requirements = setOf(this)

        val states: Set<Wrapper.OpModeState> = setOf(Wrapper.OpModeState.ACTIVE)

        return object : Command {
            var t: Double = 0.0
            val beginTs: Double = System.nanoTime() * 1e-9

            var finished: Boolean = false

            override val requirements = requirements

            override val runStates = states

            override fun toString() = "FollowTrajectoryCommand"

            override fun initialise() {}

            override fun execute() {
                t = (System.nanoTime() * 1e-9) - beginTs
                finished = followTrajectory(trajectory, t)
            }

            override fun finished(): Boolean {
                return finished
            }

            override fun end(interrupted: Boolean) {
                setDrivePowers(PoseVelocity2d(Vector2d(0.0, 0.0), 0.0))
            }
        }
    }

    fun turnCommand(turn: TimeTurn): Command {
        val requirements = setOf(this)

        val states: Set<Wrapper.OpModeState> = setOf(Wrapper.OpModeState.ACTIVE)

        return object : Command {
            var t: Double = 0.0
            val beginTs: Double = System.nanoTime() * 1e-9

            var finished: Boolean = false

            override val requirements = requirements

            override val runStates = states

            override fun toString() = "TurnCommand"

            override fun initialise() {}

            override fun execute() {
                t = (System.nanoTime() * 1e-9) - beginTs
                finished = turn(turn, t)
            }

            override fun finished(): Boolean {
                return finished
            }

            override fun end(interrupted: Boolean) {
                setDrivePowers(PoseVelocity2d(Vector2d(0.0, 0.0), 0.0))
            }
        }
    }
}