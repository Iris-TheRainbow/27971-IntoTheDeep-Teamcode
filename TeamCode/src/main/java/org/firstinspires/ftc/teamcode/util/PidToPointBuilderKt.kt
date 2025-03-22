package org.firstinspires.ftc.teamcode.util

import com.acmerobotics.roadrunner.Pose2d
import dev.frozenmilk.mercurial.commands.Command
import dev.frozenmilk.mercurial.commands.Lambda
import dev.frozenmilk.mercurial.commands.groups.Parallel
import dev.frozenmilk.mercurial.commands.groups.Sequential
import dev.frozenmilk.mercurial.commands.util.Wait
import java.util.function.DoubleSupplier
import java.util.function.Supplier
import java.util.function.UnaryOperator
import kotlin.math.tan

typealias PidToPointCommand = (x: DoubleSupplier, y: DoubleSupplier, h: DoubleSupplier, translationalAccuracy: Double, headingAccuracy: Double, evaluate: Evaluation) -> Command
typealias Instant = () -> Any
class PidToPointBuilderKt(private var pose: Pose2d, private val pidCommand: PidToPointCommand, private val currentPose: Supplier<Pose2d>) {
    private var commands = ArrayList<Command>()
    private var backup = ArrayList<Command>()
    private var repeatTimes = 0
    private var openedRepeat = false
    private var startTangent = tan(pose.heading.toDouble())
    private fun internalSetTangent(pose: Pose2d){ startTangent = pose.heading.toDouble() }
    private fun internalSetTangent(angle: Double){ startTangent = angle }

    @JvmOverloads
    fun pidAfter(seconds: Double, target: Pose2d, translationalAccuracy: Double = 3.0, headingAccuracy: Double = 1.0, eval: Evaluation = Evaluation.onInit): PidToPointBuilderKt{
        this.pose = target
        commands.add(
            Sequential(
                Wait(seconds),
                pidCommand(
                    { target.position.x },
                    { target.position.y },
                    { target.heading.toDouble() },
                    translationalAccuracy,
                    headingAccuracy,
                    eval
                )
            )
        )
        internalSetTangent(target)
        return this
    }

    @JvmOverloads
    fun pidTo(target: Pose2d, translationalAccuracy: Double = 1.0, headingAccuracy: Double = 3.0, eval: Evaluation = Evaluation.onInit): PidToPointBuilderKt{
        this.pose = target
        commands.add(
            pidCommand(
                { target.position.x },
                { target.position.y },
                { target.heading.toDouble() },
                translationalAccuracy,
                headingAccuracy,
                eval
        ))
        return this
    }

    @JvmOverloads
    fun pidRealtive(x: DoubleSupplier, y: DoubleSupplier, h: DoubleSupplier, translationalAccuracy: Double = 1.0 , headingAccuracy: Double = 3.0, eval: Evaluation = Evaluation.onInit): PidToPointBuilderKt{
        commands.add(
            pidCommand(
                { pose.position.x + x.asDouble },
                { pose.position.y + y.asDouble },
                { pose.heading.toDouble() + h.asDouble },
                translationalAccuracy,
                headingAccuracy,
                eval
            ))
        internalSetTangent(pose)
        return this
    }

    @JvmOverloads
    fun splineTo(target: Pose2d, endTangent: Double = target.heading.toDouble(),  numInt: Int = 4, translationalAccuracy: Double = 1.0, headingAccuracy: Double = 3.0, headingInteroplation: HeadingInterpolation = HeadingInterpolation.target): PidToPointBuilderKt{
        val spline = Spline(OrderedPair(this.pose.position.x, this.pose.position.y), OrderedPair(target.position.x, target.position.y), startTangent, Math.PI + endTangent)
        val curvePoints: MutableList<OrderedPair<Double>> = spline.generateSpline(numInt).getSplinePoints()
        val tangents = spline.getSplineTangents()

        val heading: UnaryOperator<Double> = when (headingInteroplation) {
            HeadingInterpolation.tangential -> UnaryOperator{ x: Double -> tangents[x.toInt()] }
            HeadingInterpolation.linear -> UnaryOperator{ x: Double -> x * ((this.pose.heading.toDouble() - target.heading.toDouble()) / numInt.toDouble()) + this.pose.heading.toDouble() }
            else -> UnaryOperator{ this.pose.heading.toDouble() }
        }

        internalSetTangent(endTangent)
        var backup = this.repeat(1)
        for (i in 0 until numInt) {
            val point = curvePoints[i]
            backup = backup.pidTo(Pose2d(point.x, point.y, heading.apply(i.toDouble())), 5.0, 999.0)
        }
        backup = backup.pidTo(Pose2d(curvePoints[numInt - 1].x, curvePoints[numInt - 1].y, heading.apply(numInt - 1.0)), translationalAccuracy, headingAccuracy)
        backup.pose = target
        return backup.stopRepeat()
    }

    @JvmOverloads
    fun splineToLinearHeading(target: Pose2d, endTangent: Double = target.heading.toDouble(),  numInt: Int = 4, translationalAccuracy: Double = 1.0, headingAccuracy: Double = 3.0): PidToPointBuilderKt{
        return splineTo(target, endTangent, numInt, translationalAccuracy, headingAccuracy, HeadingInterpolation.linear)
    }

    @JvmOverloads
    fun splineToTargetHeading(target: Pose2d, endTangent: Double = target.heading.toDouble(),  numInt: Int = 4, translationalAccuracy: Double = 1.0, headingAccuracy: Double = 3.0): PidToPointBuilderKt{
        return splineTo(target, endTangent, numInt, translationalAccuracy, headingAccuracy, HeadingInterpolation.target)
    }

    @JvmOverloads
    fun splineToTangentialHeading(target: Pose2d, endTangent: Double = target.heading.toDouble(),  numInt: Int = 4, translationalAccuracy: Double = 1.0, headingAccuracy: Double = 3.0): PidToPointBuilderKt{
        return splineTo(target, endTangent, numInt, translationalAccuracy, headingAccuracy, HeadingInterpolation.tangential)
    }

    fun stopAndAdd(vararg command: Command): PidToPointBuilderKt{
        commands.add(
            Parallel(
                *command
            ))
        return this
    }
    fun stopAndAdd(instant: Instant): PidToPointBuilderKt{
        commands.add(
                Lambda("instant").setInit(instant::invoke)
            )
        return this
    }
    fun stopAndAdd(runnable: Runnable): PidToPointBuilderKt {
        commands.add(
            Lambda("instant").setInit(runnable::run)
        )
        return this
    }

    fun afterTime(seconds: Double, vararg command: Command): PidToPointBuilderKt{
        commands[commands.size - 1] = Parallel(
            commands[commands.size - 1],
            Sequential(
                Wait(seconds),
                Parallel(
                    *command
                )
            )
        )
        return this
    }

    fun afterDisp(disp: Double, vararg command: Command): PidToPointBuilderKt{
        commands[commands.size - 1] = Parallel(
            commands[commands.size - 1],
            Sequential(
                Lambda("wait until disp").setFinish {
                    (currentPose.get().minusExp(pose).position.norm() < disp)
                },
                Parallel(
                    *command
                )
            )
        )
        return this
    }
    fun setTangent(angle: Double): PidToPointBuilderKt{ internalSetTangent(angle); return this}
    fun duringLast(vararg command: Command): PidToPointBuilderKt{ return afterTime(0.0, *command) }
    fun waitSeconds(seconds: Double): PidToPointBuilderKt{ return stopAndAdd(Wait(seconds)) }

    @JvmOverloads
    fun repeat(times: Int, xOffsetPer: Double = 0.0, yOffsetPer: Double = 0.0): PidToPointBuilderKt{
        if (times <  0) {
            throw IllegalArgumentException("Attempted to open a repeat with fewer than zero iterations")
        }
        backup = commands
        repeatTimes = times
        openedRepeat = true
        return this
    }
    fun stopRepeat(): PidToPointBuilderKt{
        if (!openedRepeat){
            throw RuntimeException("Attempted to close a repeat without one first being opened")
        }
        commands.replaceAll(commandUtil::proxiedCommand)
        var i = 0
        while (i < repeatTimes){
            backup.addAll(commands)
            i++
        }
        commands = backup
        openedRepeat = false
        return this
    }
    fun build(): Command {
        if (openedRepeat){
            throw RuntimeException("Attempted to build without closing a repeat")
        }
        commands.replaceAll(commandUtil::proxiedCommand)
        return Sequential(commands)
    }

}

enum class Evaluation {
    onInit,
    repeatedly,
    onbuild
}
enum class HeadingInterpolation{
    target,
    linear,
    tangential,
    start
}