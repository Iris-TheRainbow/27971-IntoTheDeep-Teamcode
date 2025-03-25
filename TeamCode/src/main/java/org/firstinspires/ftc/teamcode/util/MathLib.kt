package org.firstinspires.ftc.teamcode.util

import java.util.function.UnaryOperator
import kotlin.math.abs
import kotlin.math.atan2
import kotlin.math.cos
import kotlin.math.max
import kotlin.math.pow
import kotlin.math.sign
import kotlin.math.sin
import kotlin.math.sqrt
@JvmName("MathLib")

fun distance(x1: Double, y1: Double, x2: Double, y2: Double): Double {
    return (sqrt((x2 - x1).pow(2.0) + (y2 - y1).pow(2.0)))
}

fun signedFunc(func: UnaryOperator<Double>, value: Double): Double {
    return sign(value) * func.apply(abs(value))
}

fun square(x: Double): Double {
    return x * x
}

fun cube(x: Double): Double {
    return x * x * x
}

fun arcLength(func: UnaryOperator<Double>, start: Double, end: Double, subInts: Int): Double {
    var lastInput = start
    var lastOutput = func.apply(start)
    var dist = 0.0
    for (i in 0..subInts) {
        val input = i * (end - start) + start
        val output = func.apply(input)
        dist += distance(lastInput, lastOutput, input, output)
        lastOutput = output
        lastInput = input
    }
    return dist
}

class Spline(
    point0: OrderedPair<Double>,
    point1: OrderedPair<Double>,
    private val tangent0: Double,
    private val tangent1: Double) {

    private val vecTan0 = normVec(cos(tangent0), sin(tangent0))
    private val vecTan1 = normVec(cos(tangent1), sin(tangent1))
    private val magnitude = distance(point0.x, point0.y, point1.x, point1.y)

    private val splineX =
        UnaryOperator { t: Double -> (2*cube(t) - 3*square(t) + 1)*point0.x + (-2*cube(t) + 3*square(t))*point1.x + (cube(t) - 2 * square(t) + t)* magnitude * vecTan0.x + (-cube(t)+ square(t)) * magnitude * vecTan1.x }
    private val splineY =
        UnaryOperator { t: Double -> (2*cube(t) - 3*square(t) + 1)*point0.y + (-2*cube(t) + 3*square(t))*point1.y + (cube(t) - 2 * square(t) + t)* magnitude *vecTan0.y + (-cube(t)+ square(t)) * magnitude * vecTan1.y }

    private val splinePoints: MutableList<OrderedPair<Double>> = ArrayList()
    private val splineTangents: MutableList<Double> = ArrayList()
    fun getSplineTangents(): MutableList<Double>{
        return splineTangents
    }
    fun getSplinePoints(): MutableList<OrderedPair<Double>> {
        return splinePoints
    }

    fun generateSpline(numPoints: Int): Spline {
        splineTangents.add(tangent0)
        for (i in 0..numPoints) {
            splinePoints.add(
                OrderedPair(
                    splineX.apply(i * 1.0 / numPoints),
                    splineY.apply(i * 1.0 / numPoints)
                )
            )
            if (i + 1 < numPoints){
                splineTangents.add(
                    Math.tan(
                        (splineX.apply((i + 1) * 1.0 / numPoints) - splineX.apply(i * 1.0 / numPoints)) /
                        (splineY.apply((i + 1) * 1.0 / numPoints) - splineY.apply(i * 1.0 / numPoints))
                    )
                )
            }else{
                splineTangents.add(tangent1)
            }
        }
        return this
    }
}

data class OrderedPair<T>(val x: T, val y: T)
class normVec(x: Double, y: Double){
    val x = x / max(x, y)
    val y = y / max(x, y)
}