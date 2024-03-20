package org.firstinspires.ftc.teamcode.utils

import com.qualcomm.robotcore.util.ElapsedTime
import org.firstinspires.ftc.teamcode.auto.AutoVars
import org.firstinspires.ftc.teamcode.hardware.CameraControls.AutoRed
import org.firstinspires.ftc.teamcode.pp.TrajCoef
import org.firstinspires.ftc.teamcode.pp.Trajectory
import org.firstinspires.ftc.teamcode.utils.RobotFuncs.log
import org.firstinspires.ftc.teamcode.utils.RobotFuncs.send_log
import org.firstinspires.ftc.teamcode.utils.RobotVars.___KILL_DIFFY_THREADS
import java.util.Vector
import kotlin.concurrent.thread

/// -x = Goto: x
/// 1 = Trajectory
/// 2 = Function
/// 3 = SetGoto
/// 4 = Conditional
/// 5 = CondDir
// 10 = None
class TSE(val type: Int, val initActio: () -> Unit, val checkDone: () -> Boolean, val conditional: () -> Int, val traj: Trajectory?) {
    constructor(type: Int, initActio: () -> Unit, checkDone: () -> Boolean, traj: Trajectory?) : this(type, initActio, checkDone, { 0 }, traj) // Trajectory Sequence Element
    constructor(type: Int, initActio: () -> Unit, checkDone: () -> Boolean) : this(type, initActio, checkDone, { 0 }, null) // Trajectory Sequence Element
    constructor(type: Int, conditional: () -> Int) : this(type, {}, { true }, conditional, null)
    constructor(type: Int) : this(type, {}, { true }, { 0 }, null)

    override fun toString() = "($type -> (traj)$traj)"
}

class TrajectorySequence {
    private val steps = Vector<TSE>()
    private val stimer = ElapsedTime()

    fun draw() {
        for ((si, s) in steps.withIndex()) {
            if (s.type == 1) {
                if (s.traj != null) {
                    RobotFuncs.pp.drawTraj(s.traj, AutoVars.colours[si % AutoVars.colours.size])
                }
            }
        }
    }

    var curPose = Pose()
    fun addTrajectory(t: Trajectory): TrajectorySequence {
        steps.add(
                TSE(1,
                        { RobotFuncs.pp.startFollowTraj(t) },
                        { !RobotFuncs.pp.busy },
                        t
                ))
        curPose = t.end
        return this
    }

    fun addCondDir(cond: () -> Boolean, t: Trajectory): TrajectorySequence {
        steps.add(
                TSE(1,
                        { RobotFuncs.pp.startFollowTraj(t) },
                        { cond() || !RobotFuncs.pp.busy },
                        t
                ))
        return this
    }

    fun at(t: Trajectory) = addTrajectory(t)

    fun atc(tf: () -> TrajCoef, checkDone: () -> Boolean): TrajectorySequence {
        steps.add(
                TSE(1,
                        { RobotFuncs.pp.startFollowTraj(tf().t) },
                        { !RobotFuncs.pp.busy || checkDone() },
                        tf().t
                ))
        return this
    }

    fun st(goto: Int): TrajectorySequence {
        steps.add(TSE(-goto))
        log("Adding $goto", steps.size)
        return this
    }

    fun gt(cond: () -> Int): TrajectorySequence {
        steps.add(TSE(4, cond))
        return this
    }

    fun addAction(a: () -> Unit, checkDone: () -> Boolean): TrajectorySequence {
        steps.add(TSE(2, a, checkDone))
        return this
    }

    fun aa(a: () -> Unit) = addAction(a) { true }

    fun aa(a: () -> Unit, checkDone: () -> Boolean) = addAction(a, checkDone)

    fun wt(checkDone: () -> Boolean) = addAction({}, checkDone)

    private val cep = ElapsedTime()
    fun slc(s: Double, checkDone: () -> Boolean, minT: Double = 0.7) = this
            .aa { cep.reset() }
            .wt { (cep.seconds() > minT) && (cep.seconds() > s || checkDone()) }

    fun sleep(s: Double): TrajectorySequence {
        steps.add(
                TSE(2, { stimer.reset() })
                { stimer.seconds() > s }
        )
        return this
    }

    fun sl(s: Double) = sleep(s)

    private var curSteps = 0

    /**
     * @param initCommand: MATA
     */
    fun failsafeMove(afterCommand: () -> Unit, checkCommand: () -> Boolean, o1: Int, o2: Int, p1: Pose, p2: Pose): TrajectorySequence {
        val lp = curPose
        val p1p = if (AutoRed) -p1.duplicate() else p1.duplicate()
        val p2p = if (AutoRed) -p2.duplicate() else p2.duplicate()
        this
                .gt { curSteps = 0; if (checkCommand()) o2 else o1 }
                .st(o1)
                .aa(afterCommand)
                .atc({ TrajCoef(lp, lp + p1p, 1.0).st(0.2) }, checkCommand)
                //.atc({ TrajCoef(lp + p1p, lp + if (curSteps == 0) p2p else p2p.negX(), 1.0) }, checkCommand)
                .gt { ++curSteps; if (checkCommand() || curSteps == 2) o2 else o1 }
                .st(o2)
                .slc(0.3, checkCommand, 0.0)
        curPose = lp
        return this
    }

    private var curtsq: TrajectorySequence? = null
    fun failsafeAction(tseq: () -> TrajectorySequence, checkCommand: () -> Boolean, o1: Int, o2: Int): TrajectorySequence {
        this
                .gt { if (checkCommand()) o2 else o1 }
                .st(o1)
                .aa { curtsq = tseq() }
                .wt { !tseq().update() }
                .st(o2)

        return this
    }

    private var ls = 0
    private var e = TSE(10, {}, { true })

    fun reset(): TrajectorySequence {
        ls = 0
        e = TSE(10, {}, { true })
        return this
    }

    private fun runInitActio(t: TSE) {
        if (t.type == 4) {
            val resc = t.conditional()
            for (i in 0 until steps.size) {
                if (steps[i].type == -resc) {
                    ls = i
                    e = steps[ls]
                    break
                }
            }
            log("NOT FOUND AAAAAAAAAAAAAAAAAAAAAAAAAAAAa", "AAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAA  $resc")
            send_log()
            ++ls
        } else if (t.type > 0) {
            t.initActio()
        }
    }

    fun update(): Boolean {
        log("CurStep", ls)
        if (ls < steps.size) {
            if (e.type == 10) {
                ls = 0
                e = steps[0]
                runInitActio(e)
            }

            while (e.checkDone()) {
                ++ls
                if (ls < steps.size) {
                    e = steps[ls]
                    runInitActio(e)
                } else {
                    return true
                }
            }
        } else {
            return true
        }
        return false
    }

    private class Async(val t: TrajectorySequence) : Runnable {
        override fun run() {
            t.reset(); while (!t.update()) {
                Thread.sleep(2)
            }
        }
    }

    fun runAsync(): Thread {
        reset()
        val t = Thread(Async(this))
        t.setUncaughtExceptionHandler { th, er -> RobotFuncs.log("GOT ERR ASYNC ${th.id}", er.stackTraceToString()) }
        t.start()
        return t
    }

    private class AsyncDiffy(val t: TrajectorySequence) : Runnable {
        override fun run() {
            t.reset(); while (!___KILL_DIFFY_THREADS && !t.update()) {
                Thread.sleep(2)
            }
        }
    }

    fun runAsyncDiffy(): Thread {
        reset()
        val t = Thread(AsyncDiffy(this))
        t.setUncaughtExceptionHandler { th, er -> RobotFuncs.log("GOT ERR ${th.id}", er.stackTraceToString()) }
        t.start()
        return t
    }

    private fun addTSE(t: TSE): TrajectorySequence {
        steps.add(t)
        return this
    }

    fun duplicate(): TrajectorySequence {
        val ct = TrajectorySequence()
        for (s in steps) {
            ct.addTSE(s)
        }
        return ct
    }
}
