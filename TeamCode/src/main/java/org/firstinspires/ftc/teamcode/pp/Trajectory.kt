package org.firstinspires.ftc.teamcode.pp

class CubicBezierCurve(
        private val c0: Double,
        private val c1: Double,
        private val c2: Double,
        private val c3: Double) {
    /**
     * Returns the value of the polynomial at [t].
     */
    operator fun get(t: Double) =
            t * (t * (t * (-c0 + 3 * c1 - 3 * c2 + c3) + 3 * c0 - 6 * c1 + 3 * c2) - 3 * c0 + 3 * c1) + c0

    // Desmos friendly version
    // x * (x * (x * (-a + 3 * b - 3 * c + d) + 3 * a - 6 * b + 3 * c) - 3 * a + 3 * b) + a
}

class Trajectory(val start: Pose, val end: Pose, val v1e: Vec2d, val v2e: Vec2d, val h1: Vec2d) {
    constructor(sp: Pose, ep: Pose, v1x: Double, v1y: Double, v2x: Double, v2y: Double, h1x: Double, h1y: Double) : this(sp, ep, Vec2d(v1x, v1y), Vec2d(v2x, v2y), Vec2d(h1x, h1y))
    constructor(sp: Pose, ep: Pose, v1x: Double, v1y: Double, v2x: Double, v2y: Double) : this(sp, ep, Vec2d(v1x, v1y), Vec2d(v2x, v2y), Vec2d(0.3333, 0.666))
    constructor(sp: Pose, ep: Pose) : this(sp, ep, Vec2d(), Vec2d(), Vec2d(0.3333, 0.6666))

    private val v1 = v1e.polar()
    private val v2 = v2e.polar()

    private val cubX = CubicBezierCurve(start.x, v1.x, v2.x, end.x)
    private val cubY = CubicBezierCurve(start.y, v1.y, v2.y, end.y)
    private val cubH = CubicBezierCurve(0.0, h1.x, h1.y, 1.0)

    operator fun get(t: Double) = Pose(cubX[t], cubY[t], cubH[t])
}