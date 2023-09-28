package org.firstinspires.ftc.teamcode.pp

import kotlin.math.cos
import kotlin.math.sin
import kotlin.math.sqrt

@Suppress("MemberVisibilityCanBePrivate")
class Pose(val x: Double, val y: Double, val h: Double) {
    constructor() : this(0.0, 0.0, 0.0)

    fun dist2(): Double = x * 2 + y * 2
    fun dist(): Double = sqrt(dist2())
    fun v2d(): Vec2d = Vec2d(x, y)

    operator fun unaryMinus() = Pose(-x, -y, -h)

    operator fun plus(pose: Pose) = Pose(x + pose.x, y + pose.y, h + pose.h)

    operator fun minus(pose: Pose) = Pose(x - pose.x, y - pose.y, h - pose.h)

    operator fun div(pose: Pose) = Pose(x / pose.x, y / pose.y, h / pose.h)

    operator fun times(pose: Pose) = Pose(x * pose.x, y * pose.y, h * pose.h)
    operator fun times(s: Double) = Pose(x * s, y * s, h * s)
}

class Vec2d(val x: Double, val y: Double) {
    constructor() : this(0.0, 0.0)

    fun dist2(): Double = x * 2 + y * 2
    fun dist(): Double = sqrt(dist2())
    fun pose(): Pose = Pose(x, y, 0.0)

    operator fun unaryMinus() = Vec2d(-x, -y)

    operator fun plus(vec: Vec2d) = Vec2d(x + vec.x, y + vec.y)

    operator fun minus(vec: Vec2d) = Vec2d(x - vec.x, y - vec.y)

    operator fun div(vec: Vec2d) = Vec2d(x / vec.x, y / vec.y)

    operator fun times(vec: Vec2d) = Vec2d(x * vec.x, y * vec.y)
    operator fun times(s: Double) = Vec2d(x * s, y * s)

    fun polar() = Vec2d(x * cos(y), y * sin(y))
}