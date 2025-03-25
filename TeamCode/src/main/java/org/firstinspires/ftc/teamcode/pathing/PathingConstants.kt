package org.firstinspires.ftc.teamcode.pathing

import com.qualcomm.hardware.sparkfun.SparkFunOTOS
import org.firstinspires.ftc.teamcode.util.drivers.Pinpoint
import kotlin.math.PI


@JvmField
val otosOffsetX = -3.0998
@JvmField
val otosOffsetY = -3.0397
@JvmField
val otosOffsetH = -1.5672

@JvmField
val offset = SparkFunOTOS.Pose2D(otosOffsetX, otosOffsetY, otosOffsetH)

@JvmField
val otosLinearScalar = .985
@JvmField
val otosAngularScalar = .995



@JvmField
val pinpointXOffsetMM = 75.0
@JvmField
val pinpointYOffsetMM = 27.0

@JvmField
val pinpointXDirection = Pinpoint.EncoderDirection.REVERSED
@JvmField
val pinpointYDirection = Pinpoint.EncoderDirection.REVERSED

@JvmField
val axialGain = 4.0
@JvmField
val lateralGain = 4.0
@JvmField
val headingGain = 5.0
@JvmField
val axialVelGain = 0.15
@JvmField
val lateralVelGain = 0.15
@JvmField
val headingVelGain = 0.15

@JvmField
var inPerTick = 1.0
@JvmField
var lateralInPerTick = .65
@JvmField
var trackWidthTicks = 12.0

@JvmField
var kS = 1.9
@JvmField
var kV = .145
@JvmField
var kA = .05

@JvmField
var maxWheelVel = 70.0
@JvmField
var minProfileAccel = -50.0
@JvmField
var maxProfileAccel = 50.0

@JvmField
var maxAngVel = 2 * PI
@JvmField
var maxAngAccel = 1 * PI
