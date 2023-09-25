package org.firstinspires.ftc.teamcode.utils

import kotlin.math.PI
import kotlin.math.abs

object Util {
    private const val eps = 0.001
    @JvmStatic
    fun epsEq(o1: Double, o2: Double): Boolean {
        return abs(o1 - o2) < eps
    }

    @JvmStatic
    fun mod(o1: Double, o2: Double): Double {
        return (o1 + o2) % o2
    }

    @JvmStatic
    fun angDiff(o1: Double, o2: Double): Double {
        return mod(o2 - o1 + PI, PI * 2) - PI
    }
}