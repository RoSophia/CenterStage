package org.firstinspires.ftc.teamcode.pp

import com.acmerobotics.dashboard.config.Config
import org.firstinspires.ftc.teamcode.pp.DriveConstants.MAX_ACC
import org.firstinspires.ftc.teamcode.pp.DriveConstants.MAX_DEC
import org.firstinspires.ftc.teamcode.pp.DriveConstants.MAX_VEL
import kotlin.math.min
import kotlin.math.sqrt

class CubicBezierCurve(
        private val c0: Double,
        private val c1: Double,
        private val c2: Double,
        private val c3: Double) {

    operator fun get(t: Double) = t * (t * (t * (-c0 + 3 * c1 - 3 * c2 + c3) + 3 * c0 - 6 * c1 + 3 * c2) - 3 * c0 + 3 * c1) + c0
    // Desmos friendly version
    // x * (x * (x * (-a + 3 * b - 3 * c + d) + 3 * a - 6 * b + 3 * c) - 3 * a + 3 * b) + a

    fun deriv(t: Double) = t * (t * (-3 * c0 - 9 * c2 + 3 * c3) + 6 * c0 + 6 * c2) - 3 * c0 + c1 * (t * (9 * t - 12) + 3)

    fun dderiv(t: Double) = -6 * (c0 * (t - 1) - c1 * (3 * t - 2) + c2 * (3 * t - 1) - c3 * t)

    fun ddderiv(t: Double) = -6 * c0 + 18 * c1 - 18 * c2 + 6 * c3 + 0 * t
}

@Config
object DriveConstants {
    @JvmField
    var MAX_ACC = 10.0

    @JvmField
    var MAX_DEC = 10.0

    @JvmField
    var MAX_VEL = 20.0
}

class Trajectory(val start: Pose, val initVel: Double, val end: Pose, val v1e: Vec2d, val v2e: Vec2d, val h1: Vec2d, val checkpoints: Int) {
    constructor(sp: Pose, initVel: Double, ep: Pose, v1e: Vec2d, v2e: Vec2d, h1: Vec2d) : this(sp, initVel, ep, v1e, v2e, h1, 500)
    constructor(sp: Pose, initVel: Double, ep: Pose, v1x: Double, v1y: Double, v2x: Double, v2y: Double, h1x: Double, h1y: Double) : this(sp, initVel, ep, Vec2d(v1x, v1y), Vec2d(v2x, v2y), Vec2d(h1x, h1y))
    constructor(sp: Pose, initVel: Double, ep: Pose, v1x: Double, v1y: Double, v2x: Double, v2y: Double) : this(sp, initVel, ep, Vec2d(v1x, v1y), Vec2d(v2x, v2y), Vec2d(0.3333, 0.666))
    constructor(sp: Pose, initVel: Double, ep: Pose) : this(sp, initVel, ep, Vec2d(), Vec2d(), Vec2d(0.3333, 0.6666))
    constructor(sp: Pose, ep: Pose) : this(sp, 0.0, ep)
    constructor() : this(Pose(), Pose())

    private val v1 = v1e.polar()
    private val v2 = v2e.polar()
    val checkLen = 1.0 / checkpoints.toDouble()

    private val cubX = CubicBezierCurve(start.x, v1.x, v2.x, end.x)
    private val cubY = CubicBezierCurve(start.y, v1.y, v2.y, end.y)
    private val cubH = CubicBezierCurve(0.0, h1.x, h1.y, 1.0)

    operator fun get(t: Double) = if (t < 0.0) start else if (t > 1.0) end else Pose(cubX[t], cubY[t], cubH[t])
    fun deriv(t: Double) = Pose(cubX.deriv(t), cubY.deriv(t), cubH.deriv(t))

    fun getSpeed(t: Double): Pose {
        val cs = min(MAX_VEL, min(initVel + MAX_ACC * t, (1 - t) * MAX_DEC))
        val cd = deriv(t)
        return cd * cs
    }
}