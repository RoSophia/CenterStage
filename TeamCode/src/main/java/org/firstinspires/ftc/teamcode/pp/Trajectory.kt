package org.firstinspires.ftc.teamcode.pp

import org.firstinspires.ftc.teamcode.pp.PP.Checkpoints
import org.firstinspires.ftc.teamcode.pp.PP.MAX_FRACTION
import org.firstinspires.ftc.teamcode.pp.PP.MAX_TIME
import org.firstinspires.ftc.teamcode.pp.PP.PeruEnd
import org.firstinspires.ftc.teamcode.pp.PP.PeruStart
import org.firstinspires.ftc.teamcode.utils.Pose
import org.firstinspires.ftc.teamcode.utils.RobotFuncs.log
import org.firstinspires.ftc.teamcode.utils.TrajectorySequence
import org.firstinspires.ftc.teamcode.utils.Util.angDiff
import org.firstinspires.ftc.teamcode.utils.Util.angNorm
import org.firstinspires.ftc.teamcode.utils.Util.clamp
import org.firstinspires.ftc.teamcode.utils.Vec2d
import java.util.Vector

class CubicBezierCurve(
        private val c0: Double,
        private val c1: Double,
        private val c2: Double,
        private val c3: Double) {

    operator fun get(t: Double) = t * (t * (t * (-c0 + 3 * c1 - 3 * c2 + c3) + 3 * c0 - 6 * c1 + 3 * c2) - 3 * c0 + 3 * c1) + c0
    // Desmos friendly version
    // x * (x * (x * (-a + 3 * b - 3 * c + d) + 3 * a - 6 * b + 3 * c) - 3 * a + 3 * b) + a

    fun deriv(t: Double) = t * (t * (-3 * c0 - 9 * c2 + 3 * c3) + 6 * c0 + 6 * c2) - 3 * c0 + c1 * (t * (9 * t - 12) + 3)
}

class Action(val checkNr: Int, val act: () -> Unit) {
    override fun toString() = "$checkNr -> $act"
}

class TrajCoef(@JvmField var sp: Pose, @JvmField var ep: Pose, @JvmField var v1: Vec2d, @JvmField var v2: Vec2d, @JvmField var h: Vec2d, @JvmField var mf: Double, @JvmField var peru: Vec2d, var initVel: Double, var timeout: Double) {
    constructor(sp: Pose, ep: Pose, v1: Vec2d, v2: Vec2d, h: Vec2d, mf: Double, peru: Vec2d) : this(sp, ep, v1, v2, h, mf, peru, 0.0, MAX_TIME)
    constructor(sp: Pose, ep: Pose, v1: Vec2d, v2: Vec2d, h: Vec2d, mf: Double) : this(sp, ep, v1, v2, h, mf, Vec2d(PeruStart, PeruEnd))
    constructor(sp: Pose, ep: Pose, v1: Vec2d, v2: Vec2d, mf: Double) : this(sp, ep, v1, v2, Vec2d(0.0, 1.0), mf)
    constructor(sp: Pose, ep: Pose, v1: Vec2d, v2: Vec2d) : this(sp, ep, v1, v2, MAX_FRACTION)
    constructor(sp: Pose, ep: Pose, mf: Double, peru: Vec2d) : this(sp, ep, Vec2d(), Vec2d(), Vec2d(0.0, 1.0), mf, peru)
    constructor(sp: Pose, ep: Pose, mf: Double) : this(sp, ep, Vec2d(), Vec2d(), mf)
    constructor(sp: Pose, ep: Pose) : this(sp, ep, Vec2d(), Vec2d())

    constructor(ep: Pose, v1: Vec2d, v2: Vec2d, h: Vec2d, mf: Double, peru: Vec2d) : this(Pose(), ep, v1, v2, h, mf, peru)
    constructor(ep: Pose, v1: Vec2d, v2: Vec2d, h: Vec2d, mf: Double) : this(ep, v1, v2, h, mf, Vec2d(PeruStart, PeruEnd))
    constructor(ep: Pose, v1: Vec2d, v2: Vec2d, mf: Double, peru: Vec2d) : this(ep, v1, v2, Vec2d(0.0, 1.0), mf, peru)
    constructor(ep: Pose, v1: Vec2d, v2: Vec2d, mf: Double) : this(Pose(), ep, v1, v2, mf)
    constructor(ep: Pose, v1: Vec2d, v2: Vec2d) : this(Pose(), ep, v1, v2)
    constructor(ep: Pose, h: Vec2d, mf: Double) : this(Pose(), ep, Vec2d(), Vec2d(), h, mf, Vec2d(PeruStart, PeruEnd))
    constructor(ep: Pose, mf: Double, peru: Vec2d) : this(Pose(), ep, mf, peru)
    constructor(ep: Pose, mf: Double) : this(Pose(), ep, mf)
    constructor(ep: Pose) : this(Pose(), ep)
    constructor() : this(Pose())

    fun cb(): TrajCoef {
        peru = Vec2d(0.1, 0.2)
        timeout = 0.0
        return this
    }

    fun cc(): TrajCoef {
        peru = Vec2d(0.1, 0.2)
        initVel = 1000000.0
        timeout = 0.0
        return this
    }

    fun ce(): TrajCoef {
        initVel = 1000000.0
        return this
    }

    fun duplicate() = TrajCoef(sp.duplicate(), ep.duplicate(), v1.duplicate(), v2.duplicate(), h.duplicate(), mf, peru.duplicate(), initVel, timeout)
    val d: TrajCoef get() = duplicate()
    fun s(t: TrajectorySequence): TrajCoef { val tc = this.duplicate(); tc.sp = t.curPose; return tc }
    val t: Trajectory get() = Trajectory(this)
    fun st(t: Double): TrajCoef { timeout = t; return this }
    fun sx(x: Double): TrajCoef { ep.x = x; return this }
    fun so(p: Pose): TrajCoef { ep += p; return this }
    fun se(p: Pose): TrajCoef { ep = p; return this }
    fun ss(p: Pose): TrajCoef { sp += p; return this }
}

class Trajectory(val start: Pose, var initVel: Double, val end: Pose, val v1e: Vec2d, val v2e: Vec2d, val h: Vec2d, val maxFraction: Double, val peruStart: Double, val peruEnd: Double, var timeout: Double) {
    override fun toString() = "$start - $end ($v1e $v2e $h) - $maxFraction ${Vec2d(peruStart, peruEnd)}"

    constructor(tc: TrajCoef) : this(tc.sp.duplicate(), tc.initVel, tc.ep.duplicate(), tc.v1.duplicate(), tc.v2.duplicate(), tc.h.duplicate(), tc.mf, tc.peru.x, tc.peru.y, tc.timeout) /// TODO Add duplicate to all other
    constructor() : this(TrajCoef())

    private val v1 = v1e.polar()
    private val v2 = v2e.polar()
    val checkLen = 1.0 / Checkpoints.toDouble()

    private val cubX = CubicBezierCurve(start.x, start.x + v1.x, end.x + v2.x, end.x)
    private val cubY = CubicBezierCurve(start.y, start.y + v1.y, end.y + v2.y, end.y)

    private fun getHeading(v: Double) = clamp((v - h.x) / (h.y - h.x), 0.0, 1.0)

    var actions: Vector<Action> = Vector()
    var lastCompletedAction = 0

    fun addActionS(distFromStart: Double, act: () -> Unit): Trajectory {
        for (i in 0..Checkpoints) {
            if ((start - get(i)).dist() >= distFromStart) {
                actions.add(Action(i, act))
                actions.sortBy { it.checkNr }
                return this
            }
        }
        actions.add(Action(Checkpoints, act))
        actions.sortBy { it.checkNr }
        return this
    }

    fun addActionT(timeFromStart: Double, act: () -> Unit): Trajectory {
        actions.add(Action(0) { TrajectorySequence().sl(timeFromStart).aa(act).runAsync() })
        actions.sortBy { it.checkNr }
        return this
    }

    fun addActionE(distFromEnd: Double, act: () -> Unit): Trajectory {
        for (i in Checkpoints downTo 0) {
            if ((end - get(i)).dist() >= distFromEnd) {
                actions.add(Action(i, act))
                actions.sortBy { it.checkNr }
                return this
            }
        }
        actions.add(Action(0, act))
        actions.sortBy { it.checkNr }
        return this
    }

    fun nextAction(): Action =
            if (lastCompletedAction >= actions.size) {
                Action(1000000000) {}
            } else {
                actions[lastCompletedAction]
            }

    operator fun get(i: Int) = if (i < 0) start else if (i > Checkpoints) end else Pose(cubX[i * checkLen], cubY[i * checkLen], angNorm(start.h + angDiff(start.h, end.h) * getHeading(i.toDouble() * checkLen)))
    fun deriv(i: Int) = Pose(cubX.deriv(i.toDouble() * checkLen), cubY.deriv(i.toDouble() * checkLen), 0.0)
}