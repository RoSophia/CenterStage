package org.firstinspires.ftc.teamcode.auto

import com.qualcomm.robotcore.util.ElapsedTime
import org.firstinspires.ftc.teamcode.auto.AutoVars.INTAKEWAIT3
import org.firstinspires.ftc.teamcode.auto.AutoVars.WaitPreload
import org.firstinspires.ftc.teamcode.auto.AutoVars.WaitPut
import org.firstinspires.ftc.teamcode.auto.AutoVars.WaitStack1
import org.firstinspires.ftc.teamcode.auto.AutoVars.WaitStack2
import org.firstinspires.ftc.teamcode.auto.AutoVars.WaitStack3
import org.firstinspires.ftc.teamcode.auto.AutoVars.colours
import org.firstinspires.ftc.teamcode.hardware.Intakes.SIntake
import org.firstinspires.ftc.teamcode.hardware.Intakes.SInvert
import org.firstinspires.ftc.teamcode.hardware.Intakes.SKeep
import org.firstinspires.ftc.teamcode.hardware.Intakes.SPStack3
import org.firstinspires.ftc.teamcode.hardware.Intakes.SStack1
import org.firstinspires.ftc.teamcode.hardware.Intakes.SUpulLuiCostacu
import org.firstinspires.ftc.teamcode.pp.Trajectory
import org.firstinspires.ftc.teamcode.utils.Pose
import org.firstinspires.ftc.teamcode.utils.RobotFuncs.clown
import org.firstinspires.ftc.teamcode.utils.RobotFuncs.intake
import org.firstinspires.ftc.teamcode.utils.RobotFuncs.log
import org.firstinspires.ftc.teamcode.utils.RobotFuncs.pp
import org.firstinspires.ftc.teamcode.utils.RobotFuncs.slides
import org.firstinspires.ftc.teamcode.utils.RobotVars.ClownFDeschis
import org.firstinspires.ftc.teamcode.utils.RobotVars.DiffyAUp
import org.firstinspires.ftc.teamcode.utils.RobotVars.DiffyUp
import org.firstinspires.ftc.teamcode.utils.RobotVars.DiffyUpSafe
import org.firstinspires.ftc.teamcode.utils.RobotVars.GelenkCenter
import org.firstinspires.ftc.teamcode.utils.RobotVars.GelenkDif
import org.firstinspires.ftc.teamcode.utils.RobotVars.RBOT_POS
import org.firstinspires.ftc.teamcode.utils.RobotVars.RMID_POS
import org.firstinspires.ftc.teamcode.utils.RobotVars.__UPDATE_SENSORS
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
                    pp.drawTraj(s.traj, colours[si % colours.size])
                }
            }
        }
    }

    var curPose = Pose()
    fun addTrajectory(t: Trajectory): TrajectorySequence {
        steps.add(
                TSE(1,
                        { pp.startFollowTraj(t) },
                        { !pp.busy },
                        t
                ))
        curPose = t.end
        return this
    }

    fun addCondDir(cond: () -> Boolean, t: Trajectory): TrajectorySequence {
        steps.add(
                TSE(1,
                        { pp.startFollowTraj(t) },
                        { cond() || !pp.busy },
                        t
                ))
        return this
    }

    fun at(t: Trajectory) = addTrajectory(t)

    fun st(goto: Int): TrajectorySequence {
        steps.add(TSE(-goto))
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

    fun sleep(s: Double): TrajectorySequence {
        steps.add(
                TSE(2, { stimer.reset() })
                { stimer.seconds() > s }
        )
        return this
    }

    fun sl(s: Double) = sleep(s)

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
        } else if (t.type > 0) {
            t.initActio()
        }
    }

    fun update(): Boolean {
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

    fun runAsync(): Thread {
        reset()
        val t = thread {
            while (!this.update()) {
                Thread.sleep(5)
            }
        }
        t.setUncaughtExceptionHandler { _, _ -> }
        t.start()
        return t
    }

    fun runAsyncDiffy(): Thread {
        reset()
        val t = thread {
            while (!___KILL_DIFFY_THREADS && !this.update()) { /// Is interrupted fuckery
                Thread.sleep(2)
            }
            if (___KILL_DIFFY_THREADS) {
                log("KILLED ${Thread.currentThread().id}", ___KILL_DIFFY_THREADS)
            }
        }
        t.setUncaughtExceptionHandler { th, er -> log("GOT ERR ${th.id}", er.stackTraceToString()) }
        t.start()
        return t
    }

    fun addTSE(t: TSE): TrajectorySequence {
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

object Cele10Traiectorii {
    private var genTime = ElapsedTime()
    private var genTime2 = ElapsedTime()
    private var genTime3 = ElapsedTime()

    @JvmStatic
    fun getCycleTrajLong(ncycle: Int, randomCase: Int, v: LongVals): TrajectorySequence {
        val ts = TrajectorySequence()
                .aa { intake.status = SKeep }
                .at(v.aStartPreload[randomCase].st(0.6).t /// Sets timeout to 0.6
                        .addActionS(50.0) { clown.goPreloadUp() })

                .aa { clown.ghearaFar?.position = ClownFDeschis }
                .sl(WaitPreload)
                .aa { clown.goPreloadDown(); intake.status = SKeep; }

        ts.at(v.bPreloadStack[randomCase].s(ts).t
                .addActionE(40.0) { intake.status = SStack1 })
                .aa { clown.catchPixel(); __UPDATE_SENSORS = false }
                .sl(WaitStack2)

        ts.at(v.bStackBackdrop[0].s(ts).cb().t)
        ts.at(v.bStackBackdrop[1].s(ts).sx(v.cBackdropPosX[randomCase]).ce().t
                .addActionT(WaitStack3) { intake.status = SInvert }
                .addActionE(50.0) { clown.goUp(if (randomCase == 0) -1 else 1) })
                .aa { clown.open() }
                .sl(WaitPut)

        for (i in 0 until ncycle - 1) {
            ts
                    .at(v.cBackdropStack[0].s(ts).so(v.stackOffset * i).cb().t)
            ts
                    .at(v.cBackdropStack[1].s(ts).so(v.stackOffset * i).ce().st(0.6).t
                            .addActionS(0.0) { clown.goDown(); slides.setTarget(RBOT_POS) }
                            .addActionE(100.0) { intake.status = SIntake })
                    .sl(WaitStack1)
                    .aa { clown.catchPixel() }
                    .sl(WaitStack2)

            ts
                    .at(v.bStackBackdrop[0].s(ts).so(v.cBackdropOffset * i).cb().t
                            .addActionT(WaitStack3) { intake.status = SInvert })
                    .aa { clown.goUp(-2); slides.setTarget(RMID_POS) }

            ts
                    .at(v.bStackBackdrop[1].s(ts).so(v.cBackdropOffset * i).cb().t)
                    .aa { clown.open() }
                    .sl(WaitPut)
        }
        ts.at(v.zBackdropPark.s(ts).t
                .addActionS(0.0) { clown.open() }
                .addActionS(70.0) { clown.goDown(); slides.setTarget(RBOT_POS) })
                .aa { clown.gelenk?.position = GelenkCenter }
        return ts
    }

    @JvmStatic
    fun getCycleTrajShort(ncycle: Int, randomCase: Int, v: ShortVals): TrajectorySequence {
        val ts = TrajectorySequence()
                .aa { clown.targetPos = DiffyUpSafe }
                .at(v.aStartPreload[randomCase].t
                        .addActionT(0.1) { clown.goPreloadUp() }
                        .addActionS(60.0) { intake.status = SUpulLuiCostacu }) /// Go from start to put preload
                .aa { clown.ghearaFar?.position = ClownFDeschis }
                .aa { clown.targetPos = DiffyUpSafe }
                .sl(0.1)

        ts
                .at(v.aPreloadBackdrop.s(ts).sx(v.backdropPosX[randomCase]).t /// Set sp to last ep and set ep x
                        .addActionE(60.0) {
                            clown.targetPos = DiffyUp
                            clown.targetAngle = DiffyAUp
                            clown.gelenk?.position = GelenkCenter + (if (randomCase == 0) -2 else 2) * GelenkDif
                        }) /// Go from put preload to backboard
                .aa { clown.open() }
                .sl(0.1)

        for (i in 0 until ncycle) {
            /// Put -> Inter1 (Diffy down) -> Inter2 -> Stack (Diffy down + gheara inchisa -> gheara deschisa + intake)
            ts.at(v.backdropStack[0].s(ts).so(v.bStackOffset * i).cb().t /// Set sp; Add offset to ep; Set "Continue begin"; Get traj
                    .addActionS(50.0) { intake.status = SIntake; clown.goDown(); slides.setTarget(RBOT_POS) })
            ts.at(v.backdropStack[1].s(ts).so(v.bStackOffset * i).cc().t) /// Set sp (with offset from last ep carried over) ; Add offset to ep; Set "Continue Continue"; Get traj
            ts.at(v.backdropStack[2].s(ts).so(v.bStackOffset * i).cc().t) /// Set sp (with offset from last ep carried over) ; Add offset to ep; Set "Continue Continue"; Get traj
            ts.at(v.backdropStack[3].s(ts).so(v.bStackOffset * i).ce().t /// Set sp (with offset from last ep carried over) ; Add offset to ep; Set "Continue Continue"; Get traj
                    .addActionE(80.0) {
                        when (i) {
                            0 -> intake.status = SStack1
                            1 -> intake.status = SPStack3
                            else -> intake.status = SIntake
                        }
                    })

            /*
            ts.aa { genTime.reset(); __UPDATE_SENSORS = true }
            ts.addCondDir(
                    {
                        if (clown.sensorReadout() != 3) {
                            genTime2.reset()
                        }
                        (genTime2.seconds() > Min3Pixel) || (genTime.seconds() > TimeoutWaitFirstStack)
                    },
                    Trajectory(v.backdropStack.ep, 100000.0, inter4Pos.ep + Pose(-1000.0, 1000.0, 0.0), Vec2d(), Vec2d(), Vec2d(), Min3PixelSpeed)
            )
            ts.sl(0.2)
            ts.aa { clown.catchPixel(); __UPDATE_SENSORS = false }*/

            /// Stack -> Inter2 -> Inter1 -> Put (Gheara inchisa -> Diffy up + gheara deschisa)
            ts.at(v.cStackBackdrop[0].s(ts).so(v.dBackdropOffset * i).cb().t
                    .addActionT(INTAKEWAIT3) { intake.status = SInvert })
            ts.at(v.cStackBackdrop[1].s(ts).so(v.dBackdropOffset * i).cc().t)
            ts
                    .at(v.cStackBackdrop[2].s(ts).so(v.dBackdropOffset * i).ce().t
                            .addActionE(70.0) { clown.goUp(-2); slides.setTarget(RMID_POS) })
                    .aa { clown.open() }
                    .sl(WaitPut / 2.0)
        }
        ts.at(v.putPark.s(ts).t
                .addActionS(0.0) { clown.open() }
                .addActionS(50.0) { clown.goDown(); slides.setTarget(RBOT_POS) }
        )
        return ts
    }
}