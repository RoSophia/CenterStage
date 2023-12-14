package org.firstinspires.ftc.teamcode.utils

import kotlin.math.PI
import kotlin.math.abs
import kotlin.math.floor
import kotlin.math.max
import kotlin.math.min

object Util {
    const val eps = 0.001
    @JvmStatic
    fun epsEq(o1: Double, o2: Double): Boolean {
        return abs(o1 - o2) < eps
    }

    @JvmStatic
    fun mod(o1: Double, o2: Double): Double {
        return (o1 + o2) % o2
    }

    @JvmStatic
    fun floatMod(o1: Double, o2: Double): Double {
        return o1 - floor(o1 / o2) * o2
    }

    @JvmStatic
    fun angNorm(o1: Double): Double {
        return floatMod(o1, 2 * PI)
    }

    @JvmStatic
    fun angDiff(o1: Double, o2: Double): Double {
        return floatMod(o2 - o1 + PI, PI * 2) - PI
    }

    @JvmStatic
    fun clamp(v: Double, down: Double, up: Double): Double {
        return min(max(v, down), up)
    }
}