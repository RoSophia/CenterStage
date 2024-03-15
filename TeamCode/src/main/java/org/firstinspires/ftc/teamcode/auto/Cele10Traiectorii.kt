package org.firstinspires.ftc.teamcode.auto

import com.qualcomm.robotcore.util.ElapsedTime
import org.firstinspires.ftc.teamcode.auto.AutoVars.INTAKEWAIT3
import org.firstinspires.ftc.teamcode.auto.AutoVars.WaitPreload
import org.firstinspires.ftc.teamcode.auto.AutoVars.WaitPut
import org.firstinspires.ftc.teamcode.auto.AutoVars.WaitStack1
import org.firstinspires.ftc.teamcode.auto.AutoVars.WaitStack2
import org.firstinspires.ftc.teamcode.auto.AutoVars.WaitStack3
import org.firstinspires.ftc.teamcode.auto.AutoVars.colours
import org.firstinspires.ftc.teamcode.hardware.CameraControls.AutoRed
import org.firstinspires.ftc.teamcode.hardware.Intakes.SIntake
import org.firstinspires.ftc.teamcode.hardware.Intakes.SInvert
import org.firstinspires.ftc.teamcode.hardware.Intakes.SKeep
import org.firstinspires.ftc.teamcode.hardware.Intakes.SPStack3
import org.firstinspires.ftc.teamcode.hardware.Intakes.SStack1
import org.firstinspires.ftc.teamcode.hardware.Intakes.SStack2
import org.firstinspires.ftc.teamcode.hardware.Intakes.SStack3
import org.firstinspires.ftc.teamcode.hardware.Intakes.SStack4
import org.firstinspires.ftc.teamcode.hardware.Intakes.SStack5
import org.firstinspires.ftc.teamcode.hardware.Intakes.SUpulLuiCostacu
import org.firstinspires.ftc.teamcode.pp.Trajectory
import org.firstinspires.ftc.teamcode.utils.Pose
import org.firstinspires.ftc.teamcode.utils.RobotFuncs.clown
import org.firstinspires.ftc.teamcode.utils.RobotFuncs.etime
import org.firstinspires.ftc.teamcode.utils.RobotFuncs.intake
import org.firstinspires.ftc.teamcode.utils.RobotFuncs.log
import org.firstinspires.ftc.teamcode.utils.RobotFuncs.lom
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
        //log("Added ${-goto} at", steps.size)
        return this
    }

    fun gt(cond: () -> Int): TrajectorySequence {
        steps.add(TSE(4, cond))
        //log("Added goto at", steps.size)
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
        //log("Running initial action for", t)
        if (t.type == 4) {
            return;
            val resc = t.conditional()
            //log("Got conditional result", resc)
            for (i in 0 until steps.size) {
                if (steps[i].type == -resc) {
                    ls = i
                    e = steps[ls]
                    //log("Found step at ", "$i -> $e")
                    break
                }
            }
            ++ls
            e = steps[ls]
        } else if (t.type > 0) {
            t.initActio()
        }
    }

    fun update(): Boolean {
        //log("Running Update ", "$ls -> ${String.format("%.4f", etime.seconds())}")
        if (ls < steps.size) {
            if (e.type == 10) {
                ls = 0
                e = steps[0]
                runInitActio(e)
            }

            //log("Currently running ", "$e -> ${String.format("%.4f", etime.seconds())} and ${e.checkDone()}")
            while (e.checkDone() && !lom.isStopRequested) {
                //log("Finished $ls ", "$e -> ${String.format("%.4f", etime.seconds())}")
                ++ls
                if (ls < steps.size) {
                    try {
                        e = steps[ls]
                        runInitActio(e)
                    } catch (e: Exception) {
                        log("AAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAA KILL error", e.stackTraceToString())
                        log("AAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAA KILL2 ERROR", "${ls}:${steps.size}")
                    }
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
        for ((ei, e) in steps.withIndex()) {
            //log("Step $ei ->", e)
        }
        reset()
        val t = thread { while (!this.update() && Thread.currentThread().isAlive) { Thread.sleep(5) }; /*log("KMSKMSKM DONE", "AAAAA")*/ }
        t.setUncaughtExceptionHandler { th, er -> log("GOT ERR ${th.id}", er.stackTraceToString()) }
        t.start()
        return t
    }

    fun runAsyncDiffy(): Thread {
        reset()
        val t = thread {
            while (!___KILL_DIFFY_THREADS && !this.update() && Thread.currentThread().isAlive) { /// Is interrupted fuckery
                Thread.sleep(2)
            }
            if (___KILL_DIFFY_THREADS) {
                log("KILLED ${Thread.currentThread().id}", ___KILL_DIFFY_THREADS)
            }
        }
        t.setUncaughtExceptionHandler { th, er -> log("GOT ERR ${th.id}", er.stackTraceToString())
            log("GOT ERR2 ${th.id}", er.message ?: "")
        }
        t.start()
        return t
    }

    private fun addTSE(t: TSE): TrajectorySequence {
        steps.add(t)
        return this
    }
}

object Cele10Traiectorii {
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
                .addActionE(40.0) { intake.status = SStack5 })
                .sl(WaitStack1)
                .aa { clown.catchPixel(); __UPDATE_SENSORS = false }
                .sl(WaitStack2)


        ts.at(v.bStackBackdrop[0].s(ts).cb().t)
        ts.at(v.bStackBackdrop[1].s(ts).sx(v.cBackdropPosX[randomCase]).ce().t
                .addActionT(WaitStack3) { intake.status = SInvert }
                .addActionE(50.0) { clown.goUp(if (randomCase == if (AutoRed) 0 else 2) -1 else 1) })
                .aa { clown.open() }
                .sl(WaitPut)

        for (i in 0 until ncycle - 1) {
            ts
                    .at(v.cBackdropStack[0].s(ts).so(v.stackOffset * i).cb().t)
            ts
                    .at(v.cBackdropStack[1].s(ts).so(v.stackOffset * i).ce().st(0.6).t
                            .addActionS(0.0) { clown.goDown(); slides.setTarget(RBOT_POS) }
                            .addActionE(100.0) { intake.status = if (i == 0) SStack3 else SIntake })
                    .sl(WaitStack1)
                    .aa { clown.catchPixel() }
                    .sl(WaitStack2)

            ts
                    .at(v.bStackBackdrop[0].s(ts).so(v.cBackdropOffset * i).cb().t
                            .addActionT(WaitStack3) { intake.status = SInvert })
            ts
                    .at(v.bStackBackdrop[1].s(ts).so(v.cBackdropOffset * i).cb().t
                            .addActionE(80.0) { clown.goUp(-1); slides.setTarget(RMID_POS) })
                    .aa { clown.open() }
                    .sl(WaitPut)
        }

        ts.at(v.zBackdropPark.s(ts).t
                .addActionS(0.0) { clown.open() }
                .addActionS(70.0) { clown.goDown(); slides.setTarget(RBOT_POS) })
                .aa { clown.gelenk?.position = GelenkCenter }
        return ts
    }

    fun gi(i: Int, p: Int) =  (i + 1) * 20 + p

    @JvmStatic
    fun getCycleTrajShort(ncycle: Int, randomCase: Int, v: ShortVals): TrajectorySequence {
        val ts = TrajectorySequence()
                .st(4)
                .aa { clown.targetPos = DiffyUpSafe }
                .at(v.aStartPreload[randomCase].t
                        .addActionT(0.1) { clown.goPreloadUp() }
                        .addActionS(60.0) { intake.status = SUpulLuiCostacu }) /// Go from start to put preload
                .aa { clown.ghearaFar?.position = ClownFDeschis }
                .aa { clown.targetPos = DiffyUpSafe }
                .sl(0.1)
                //.gt {4}
        ts
                .at(v.aPreloadBackdrop.s(ts).sx(v.backdropPosX[randomCase]).t /// Set sp to last ep and set ep x
                        .addActionE(60.0) {
                            clown.targetPos = DiffyUp
                            clown.targetAngle = DiffyAUp
                            clown.curState = 0
                            clown.gelenk?.position = GelenkCenter + (if (randomCase == (if (AutoRed) 0 else 2)) -1 else 1) * GelenkDif
                        }) /// Go from put preload to backboard
                .aa { clown.open() }
                .sl(0.1)

        for (i in 0 until ncycle) {
            /// Put -> Inter1 (Diffy down) -> Inter2 -> Stack (Diffy down + gheara inchisa -> gheara deschisa + intake)
            ts.at(v.backdropStack[0].s(ts).so(v.bStackOffset * i).cb().t /// Set sp; Add offset to ep; Set "Continue begin"; Get traj
                    .addActionS(50.0) { intake.status = SIntake; clown.goDown(); slides.setTarget(RBOT_POS) })
            ts.at(v.backdropStack[1].s(ts).so(v.bStackOffset * i).cc().t) /// Set sp (with offset from last ep carried over) ; Add offset to ep; Set "Continue Continue"; Get traj
            ts.at(v.backdropStack[2].s(ts).so(v.bStackOffset * i).cc().t) /// Set sp (with offset from last ep carried over) ; Add offset to ep; Set "Continue Continue"; Get traj
                    .aa { __UPDATE_SENSORS = true }
            ts.at(v.backdropStack[3].s(ts).so(v.bStackOffset * i).ce().t /// Set sp (with offset from last ep carried over) ; Add offset to ep; Set "Continue Continue"; Get traj
                    .addActionE(80.0) {
                        when (i) {
                            0 -> intake.status = SStack4
                            1 -> intake.status = SStack2
                            else -> intake.status = SIntake
                        }
                    })
                    .sl(WaitStack1)

            /*
            ts.st(gi(i, 1))
            ts.gt { if (clown.sensorReadout() == 3) 10 else 2 }
            ts.st(gi(i, 2))
            ts.at(v.bbTryStackAgain.s(ts).so(v.bStackOffset * i).ce().t)
            ts.gt { 1 }
            ts.st(gi(i, 10))*/
                    .aa { clown.catchPixel() }
                    .sl(WaitStack2)



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