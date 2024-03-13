package org.firstinspires.ftc.teamcode.auto

import com.qualcomm.robotcore.util.ElapsedTime
import org.firstinspires.ftc.teamcode.auto.AutoVars.Min3Pixel
import org.firstinspires.ftc.teamcode.auto.AutoVars.Min3PixelSpeed
import org.firstinspires.ftc.teamcode.auto.AutoVars.TimeoutWaitFirstStack
import org.firstinspires.ftc.teamcode.auto.AutoVars.WaitPreload
import org.firstinspires.ftc.teamcode.auto.AutoVars.WaitPut
import org.firstinspires.ftc.teamcode.auto.AutoVars.WaitStack1
import org.firstinspires.ftc.teamcode.auto.AutoVars.WaitStack2
import org.firstinspires.ftc.teamcode.auto.AutoVars.WaitStack3
import org.firstinspires.ftc.teamcode.auto.AutoVars.colours
import org.firstinspires.ftc.teamcode.auto.BlueLongP.bPAfterAfterShave
import org.firstinspires.ftc.teamcode.auto.BlueLongP.bPAfterShave
import org.firstinspires.ftc.teamcode.auto.BlueLongP.bPPos
import org.firstinspires.ftc.teamcode.auto.BlueLongP.bPStack
import org.firstinspires.ftc.teamcode.auto.BlueLongP.bParkPos
import org.firstinspires.ftc.teamcode.auto.BlueLongP.bPut1Pos
import org.firstinspires.ftc.teamcode.auto.BlueLongP.bPutOffset
import org.firstinspires.ftc.teamcode.auto.BlueLongP.bPutPos
import org.firstinspires.ftc.teamcode.auto.BlueLongP.bPutXCase
import org.firstinspires.ftc.teamcode.auto.BlueLongP.bPutY
import org.firstinspires.ftc.teamcode.auto.BlueLongP.bPutYOffsetCase
import org.firstinspires.ftc.teamcode.auto.BlueLongP.bStackOffset
import org.firstinspires.ftc.teamcode.auto.BlueLongP.bStackPos12
import org.firstinspires.ftc.teamcode.auto.BlueLongP.bStackPos2
import org.firstinspires.ftc.teamcode.auto.BlueShortP.sbTrajPreload2Mid
import org.firstinspires.ftc.teamcode.auto.BlueShortP.sbParkPos
import org.firstinspires.ftc.teamcode.auto.BlueShortP.sbTrajMid2BackBoard
import org.firstinspires.ftc.teamcode.auto.BlueShortP.sbPutOffset
import org.firstinspires.ftc.teamcode.auto.BlueShortP.sbPutXCase
import org.firstinspires.ftc.teamcode.auto.BlueShortP.sbPutYOffsetCase
import org.firstinspires.ftc.teamcode.auto.BlueShortP.sbStackOffset
import org.firstinspires.ftc.teamcode.auto.BlueShortP.sbTrajBack2Sstack
import org.firstinspires.ftc.teamcode.auto.BlueShortP.sbTrajStack2Back
import org.firstinspires.ftc.teamcode.auto.RedLongP.rPAfterAfterShave
import org.firstinspires.ftc.teamcode.auto.RedLongP.rPAfterShave
import org.firstinspires.ftc.teamcode.auto.RedLongP.rPPos
import org.firstinspires.ftc.teamcode.auto.RedLongP.rPStack
import org.firstinspires.ftc.teamcode.auto.RedLongP.rParkPos
import org.firstinspires.ftc.teamcode.auto.RedLongP.rPut1Pos
import org.firstinspires.ftc.teamcode.auto.RedLongP.rPutOffset
import org.firstinspires.ftc.teamcode.auto.RedLongP.rPutPos
import org.firstinspires.ftc.teamcode.auto.RedLongP.rPutXCase
import org.firstinspires.ftc.teamcode.auto.RedLongP.rPutY
import org.firstinspires.ftc.teamcode.auto.RedLongP.rPutYOffsetCase
import org.firstinspires.ftc.teamcode.auto.RedLongP.rStackOffset
import org.firstinspires.ftc.teamcode.auto.RedLongP.rStackPos12
import org.firstinspires.ftc.teamcode.auto.RedLongP.rStackPos2
import org.firstinspires.ftc.teamcode.auto.RedShortP.srPPos
import org.firstinspires.ftc.teamcode.auto.RedShortP.srParkPos
import org.firstinspires.ftc.teamcode.auto.RedShortP.srPutFromPreloadPos
import org.firstinspires.ftc.teamcode.auto.RedShortP.srPutOffset
import org.firstinspires.ftc.teamcode.auto.RedShortP.srPutXCase
import org.firstinspires.ftc.teamcode.auto.RedShortP.srPutYOffsetCase
import org.firstinspires.ftc.teamcode.auto.RedShortP.srStackOffset
import org.firstinspires.ftc.teamcode.auto.RedShortP.srStackPPose
import org.firstinspires.ftc.teamcode.auto.RedShortP.srStackPPut
import org.firstinspires.ftc.teamcode.hardware.Intakes.SIntake
import org.firstinspires.ftc.teamcode.hardware.Intakes.SInvert
import org.firstinspires.ftc.teamcode.hardware.Intakes.SInvert2
import org.firstinspires.ftc.teamcode.hardware.Intakes.SKeep
import org.firstinspires.ftc.teamcode.hardware.Intakes.SStack1
import org.firstinspires.ftc.teamcode.hardware.Intakes.SStack2
import org.firstinspires.ftc.teamcode.hardware.Intakes.SStack3
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
import org.firstinspires.ftc.teamcode.utils.RobotVars.INTAKEWAIT1
import org.firstinspires.ftc.teamcode.utils.RobotVars.INTAKEWAIT2
import org.firstinspires.ftc.teamcode.utils.RobotVars.INTAKEWAIT3
import org.firstinspires.ftc.teamcode.utils.RobotVars.INTAKEWAIT4
import org.firstinspires.ftc.teamcode.utils.RobotVars.RBOT_POS
import org.firstinspires.ftc.teamcode.utils.RobotVars.RMID_POS
import org.firstinspires.ftc.teamcode.utils.RobotVars.__UPDATE_SENSORS
import org.firstinspires.ftc.teamcode.utils.RobotVars.___KILL_DIFFY_THREADS
import org.firstinspires.ftc.teamcode.utils.Vec2d
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

    fun addTrajectory(t: Trajectory): TrajectorySequence {
        steps.add(
                TSE(1,
                        { pp.startFollowTraj(t) },
                        { !pp.busy },
                        t
                ))
        return this
    }

    fun addCondDir(cond: ()->Boolean, t: Trajectory): TrajectorySequence {
        steps.add(
                TSE(1,
                        { pp.startFollowTraj(t) },
                        {cond() || !pp.busy},
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
                Thread.sleep(5)
            }
            if (___KILL_DIFFY_THREADS) {
                log("KILLED ${Thread.currentThread().id}", ___KILL_DIFFY_THREADS)
            }
        }
        t.setUncaughtExceptionHandler { _, _ -> }
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
    var genTime = ElapsedTime()
    var genTime2 = ElapsedTime()

    @JvmStatic
    fun getCycleTrajLongBlue(ncycle: Int, randomCase: Int): TrajectorySequence {
        val preloadPos = bPPos[randomCase].duplicate()
        preloadPos.timeout = 0.6
        val preloadTraj = Trajectory(preloadPos)

        val stackPos = bPStack[randomCase].duplicate()
        stackPos.sp = preloadPos.ep
        stackPos.setNoEndBegin()
        val stackTraj = Trajectory(stackPos)

        val afterShavePos = bPAfterShave.duplicate()
        afterShavePos.sp = stackPos.ep
        stackPos.setNoEndEnd()
        val afterShaveTraj = Trajectory(afterShavePos)

        val put1Pos = bPut1Pos.duplicate()
        put1Pos.sp = afterShavePos.ep
        put1Pos.setNoEndBegin()
        val put1Traj = Trajectory(put1Pos)

        val putPos = bPutPos.duplicate()
        putPos.sp = put1Pos.ep
        putPos.ep.y += bPutYOffsetCase[randomCase]
        putPos.ep.x = bPutXCase[randomCase]
        put1Pos.setNoEndEnd()
        var putTraj = Trajectory(putPos)

        val stack12Pos = bStackPos12.duplicate()
        stack12Pos.setNoEndBegin()
        val stack2Pos = bStackPos2.duplicate()
        stack2Pos.setNoEndEnd()
        val afterAfterShavePos = bPAfterAfterShave.duplicate()
        afterAfterShavePos.sp.y = afterAfterShavePos.ep.y

        val parkPos = bParkPos.duplicate()
        parkPos.sp = putPos.ep
        val parkTraj = Trajectory(parkPos)

        val ts = TrajectorySequence()
        /// Start -> Go to preload (Spit out pixel)
        ts.aa { intake.status = SKeep }
        preloadTraj.addActionS(50.0) { clown.goPreloadUp() }
        ts.at(preloadTraj)

        ts.aa { clown.ghearaFar?.position = ClownFDeschis }
        /// Preload -> Stack (Gheara deschisa + intake)
        ts.sl(WaitPreload)
        ts.aa { clown.goPreloadDown(); intake.status = SKeep; }
        stackTraj.addActionE(40.0) { intake.status = SStack1 }
        ts.at(stackTraj)

        //ts.sl(0.1)
        ts.aa { intake.status = SStack3 }
        ts.aa { genTime.reset(); __UPDATE_SENSORS = true }
        ts.addCondDir(
                {
                    if (clown.sensorReadout() != 3) { genTime2.reset() }
                    (genTime2.seconds() > Min3Pixel) || (genTime.seconds() > TimeoutWaitFirstStack)
                },
                Trajectory(stackPos.ep, 100000.0, stackPos.ep + Pose(0.0, 100.0, 0.0), Vec2d(), Vec2d(), Vec2d(), Min3PixelSpeed)
        )
        ts.sl(0.2)
        ts.aa { clown.catchPixel(); __UPDATE_SENSORS = false }
        ts.sl(WaitStack2)
        put1Traj.addActionT(WaitStack3) { intake.status = SInvert }
        put1Traj.addActionE(20.0) { clown.goUp(if (randomCase == 2) -2 else 2) }
        ts.at(put1Traj)

        putTraj.initVel = 10000000.0
        ts.at(putTraj)

        ts.sl(WaitPut)
        ts.aa { clown.open() }
        ts.sl(WaitPut)

        for (i in 0 until ncycle - 1) {
            /// Put -> Stack2 (Diffy down + gheara inchisa -> gheara deschisa + intake)
            stack12Pos.sp = putPos.ep
            val stack12Traj = Trajectory(stack12Pos)
            ts.at(stack12Traj)
            stack2Pos.sp = stack12Pos.ep
            stack2Pos.ep = bStackPos2.ep + bStackOffset * i
            val stack2Traj = Trajectory(stack2Pos)
            stack2Traj.addActionS(0.0) { clown.goDown(); slides.setTarget(RBOT_POS) }
            stack2Traj.addActionE(100.0) { intake.status = SIntake }
            stack2Traj.timeout = 0.0
            //stack2Traj.addActionE(50.0) { intake.status = if (i == 0) SStack2 else SIntake }

            ts.at(stack2Traj)

            afterAfterShavePos.sp = stack2Pos.ep
            afterAfterShavePos.initVel = 100000000.0
            val afterAfterShaveTraj = Trajectory(afterAfterShavePos)
            ts.at(afterAfterShaveTraj)

            /// Stack2 -> Put (Gheara inchisa -> Diffy up + gheara deschisa)
            ts.sl(WaitStack1)
            ts.aa { clown.catchPixel() }
            ts.sl(WaitStack2)

            val newPut1Traj = Trajectory(put1Pos)
            newPut1Traj.addActionT(WaitStack3) { intake.status = SInvert }
            ts.at(newPut1Traj)
            ts.aa { clown.goUp(-2); slides.setTarget(RMID_POS) }

            putPos.sp = put1Pos.ep
            putPos.ep.x = bPutXCase[3]
            putPos.ep.y = bPutY
            putPos.ep += bPutOffset * i
            putTraj = Trajectory(putPos)
            putTraj.initVel = 10000000.0
            ts.at(putTraj)

            ts.sl(WaitPut)
            ts.aa { clown.open() }
            ts.sl(WaitPut)
        }
        /// Put -> Park
        parkTraj.addActionS(0.0) { clown.open() }
        parkTraj.addActionS(70.0) { clown.goDown(); slides.setTarget(RBOT_POS) }
        ts.at(parkTraj)
        ts.aa { clown.gelenk?.position = GelenkCenter }
        return ts
    }

    @JvmStatic
    fun getCycleTrajLongRed(ncycle: Int, randomCase: Int): TrajectorySequence {
        val preloadPos = rPPos[randomCase].duplicate()
        preloadPos.timeout = 0.6
        val preloadTraj = Trajectory(preloadPos)

        val stackPos = rPStack[randomCase].duplicate()
        stackPos.sp = preloadPos.ep
        val stackTraj = Trajectory(stackPos)

        val afterShavePos = rPAfterShave.duplicate()
        afterShavePos.sp = stackPos.ep
        stackPos.setNoEndEnd()
        val afterShaveTraj = Trajectory(afterShavePos)

        val put1Pos = rPut1Pos.duplicate()
        put1Pos.sp = afterShavePos.ep
        put1Pos.setNoEndBegin()
        val put1Traj = Trajectory(put1Pos)

        val putPos = rPutPos.duplicate()
        putPos.sp = put1Pos.ep
        putPos.ep.y += rPutYOffsetCase[randomCase]
        putPos.ep.x = rPutXCase[randomCase]
        put1Pos.setNoEndEnd()
        var putTraj = Trajectory(putPos)

        val stack12Pos = rStackPos12.duplicate()
        stack12Pos.setNoEndBegin()
        val stack2Pos = rStackPos2.duplicate()
        stack2Pos.setNoEndEnd()
        val afterAfterShavePos = rPAfterAfterShave.duplicate()
        afterAfterShavePos.sp.y = afterAfterShavePos.ep.y

        val parkPos = rParkPos.duplicate()
        parkPos.sp = putPos.ep
        val parkTraj = Trajectory(parkPos)

        val ts = TrajectorySequence()
        /// Start -> Go to preload (Spit out pixel)
        ts.aa { intake.status = SKeep }
        preloadTraj.addActionS(50.0) { clown.goPreloadUp() }
        ts.at(preloadTraj)

        ts.aa { clown.ghearaFar?.position = ClownFDeschis }
        /// Preload -> Stack (Gheara deschisa + intake)
        ts.sl(WaitPreload)
        ts.aa { clown.goPreloadDown(); intake.status = SKeep; }
        stackTraj.addActionE(40.0) { intake.status = SStack1 }
        ts.at(stackTraj)

        ts.sl(0.2)
        ts.aa { intake.status = SStack3 }
        ts.aa { genTime.reset(); __UPDATE_SENSORS = true }
        ts.addCondDir(
                {
                    if (clown.sensorReadout() != 3) { genTime2.reset() }
                    (genTime2.seconds() > Min3Pixel) || (genTime.seconds() > TimeoutWaitFirstStack)
                },
                Trajectory(stackPos.ep, 100000.0, stackPos.ep + Pose(0.0, -100.0, 0.0), Vec2d(), Vec2d(), Vec2d(), Min3PixelSpeed)
        )
        ts.sl(0.2)
        ts.aa { clown.catchPixel(); __UPDATE_SENSORS = false }
        ts.sl(WaitStack2)
        put1Traj.addActionT(WaitStack3) { intake.status = SInvert2 }
        put1Traj.addActionE(20.0) { clown.goUp(if (randomCase == 2) 2 else 2) }
        ts.at(put1Traj)

        putTraj.initVel = 10000000.0
        ts.at(putTraj)

        ts.sl(WaitPut)
        ts.aa { clown.open() }
        ts.sl(WaitPut)

        for (i in 0 until ncycle - 1) {
            /// Put -> Stack2 (Diffy down + gheara inchisa -> gheara deschisa + intake)
            stack12Pos.sp = putPos.ep
            val stack12Traj = Trajectory(stack12Pos)
            ts.at(stack12Traj)
            stack2Pos.sp = stack12Pos.ep
            stack2Pos.ep = rStackPos2.ep + rStackOffset * i
            val stack2Traj = Trajectory(stack2Pos)
            stack2Traj.addActionS(0.0) { clown.goDown(); slides.setTarget(RBOT_POS) }
            stack2Traj.addActionE(100.0) { intake.status = SIntake }
            stack2Traj.timeout = 0.0
            //stack2Traj.addActionE(50.0) { intake.status = if (i == 0) SStack2 else SIntake }

            ts.at(stack2Traj)

            afterAfterShavePos.sp = stack2Pos.ep
            afterAfterShavePos.initVel = 100000000.0
            val afterAfterShaveTraj = Trajectory(afterAfterShavePos)
            ts.at(afterAfterShaveTraj)

            /// Stack2 -> Put (Gheara inchisa -> Diffy up + gheara deschisa)
            ts.sl(WaitStack1)
            ts.aa { clown.catchPixel() }
            ts.sl(WaitStack2)

            val newPut1Traj = Trajectory(put1Pos)
            newPut1Traj.addActionT(WaitStack3) { intake.status = SInvert2 }
            ts.at(newPut1Traj)
            ts.aa { clown.goUp(-2); slides.setTarget(RMID_POS) }

            putPos.sp = put1Pos.ep
            putPos.ep.x = rPutXCase[3]
            putPos.ep.y = rPutY
            putPos.ep += rPutOffset * i
            putTraj = Trajectory(putPos)
            putTraj.initVel = 10000000.0
            ts.at(putTraj)

            ts.sl(WaitPut)
            ts.aa { clown.open() }
            ts.sl(WaitPut)
        }
        /// Put -> Park
        parkTraj.addActionS(0.0) { clown.open() }
        parkTraj.addActionS(50.0) { clown.goDown(); slides.setTarget(RBOT_POS) }
        ts.at(parkTraj)
        ts.aa { clown.gelenk?.position = GelenkCenter }
        return ts
    }

    @JvmStatic
    fun getCycleTrajShortBlue(ncycle: Int, randomCase: Int): TrajectorySequence {
        val preloadPos = sbTrajPreload2Mid[randomCase].duplicate()
        val preloadTraj = Trajectory(preloadPos)

        val putFromPreloadPos = sbTrajMid2BackBoard.duplicate()
        putFromPreloadPos.sp = preloadPos.ep
        putFromPreloadPos.ep.y += sbPutYOffsetCase[randomCase]
        putFromPreloadPos.ep.x = sbPutXCase[randomCase]
        val putFromPreloadTraj = Trajectory(putFromPreloadPos)


        val ts = TrajectorySequence()
        /// Start (DiffyPreloadUp) -> Go to preload (DiffyOpenFar)
        ts.aa { clown.targetPos = DiffyUpSafe }
        preloadTraj.addActionT(0.1) { clown.goPreloadUp() }
        preloadTraj.addActionS(60.0) { intake.status = SUpulLuiCostacu }
        ts.at(preloadTraj)
        ts.aa { clown.ghearaFar?.position = ClownFDeschis }
        /// Preload -> Put (DiffyUp)
        ts.aa { clown.targetPos = DiffyUpSafe }
        putFromPreloadTraj.addActionE(60.0) {
            clown.targetPos = DiffyUp
            clown.targetAngle = DiffyAUp
            clown.gelenk?.position = GelenkCenter + 2 * GelenkDif
        }
        ts.sleep(0.2)
        ts.at(putFromPreloadTraj)
        ts.aa { clown.open() }
        ts.sl(0.1)
        for (i in 0 until ncycle) {
            /// Put -> Inter1 (Diffy down) -> Inter2 -> Stack (Diffy down + gheara inchisa -> gheara deschisa + intake)
            val inter1Pos = sbTrajBack2Sstack[0].duplicate()
            inter1Pos.sp = putFromPreloadPos.ep
            inter1Pos.ep += sbStackOffset * i
            inter1Pos.setNoEndBegin()
            val inter2Pos = sbTrajBack2Sstack[1].duplicate()
            inter2Pos.sp = inter1Pos.ep
            inter2Pos.ep += sbStackOffset * i
            inter2Pos.setNoEndContinue()
            val inter3Pos = sbTrajBack2Sstack[2].duplicate()
            inter3Pos.sp = inter2Pos.ep
            inter3Pos.ep += sbStackOffset * i
            inter3Pos.setNoEndContinue()
            val inter4Pos = sbTrajBack2Sstack[3].duplicate()
            inter4Pos.sp = inter3Pos.ep
            inter4Pos.sp.y = inter4Pos.ep.y
            inter4Pos.ep += sbStackOffset * i
            inter4Pos.setNoEndEnd()

            val put1Pos = sbTrajStack2Back[0].duplicate()
            put1Pos.sp = inter4Pos.ep
            put1Pos.ep += sbPutOffset * i
            put1Pos.setNoEndBegin()
            val put2Pos = sbTrajStack2Back[1].duplicate()
            put2Pos.sp = put1Pos.ep
            put2Pos.ep += sbPutOffset * i
            put2Pos.setNoEndContinue()
            val put3Pos = sbTrajStack2Back[2].duplicate()
            put3Pos.sp = put2Pos.ep
            put3Pos.ep += sbPutOffset * i
            put3Pos.setNoEndEnd()

            val inter1Traj = Trajectory(inter1Pos)
            inter1Traj.addActionS(60.0) { intake.status = SIntake; clown.goDown(); slides.setTarget(RBOT_POS) }
            ts.at(inter1Traj)

            val inter2Traj = Trajectory(inter2Pos)
            ts.at(inter2Traj)

            val inter3Traj = Trajectory(inter3Pos)
            ts.at(inter3Traj)

            val inter4Traj = Trajectory(inter4Pos)
            if (i == 0) {
                inter4Traj.addActionE(80.0) { intake.status = SStack1 }
                ts.at(inter4Traj)
                ts.aa { intake.status = SStack2 }
                ts.sl(INTAKEWAIT1)
                ts.aa { intake.status = SStack3 }
                ts.sl(INTAKEWAIT2)
            } else {
                ts.at(inter4Traj)
                ts.sl(INTAKEWAIT4)
            }
            ts.aa { clown.catchPixel() }
            // val inter4Traj = Trajectory(inter4Pos)
            // ts.at(inter4Traj)

            /// Stack -> Inter2 -> Inter1 -> Put (Gheara inchisa -> Diffy up + gheara deschisa)
            val put1Traj = Trajectory(put1Pos)
            put1Traj.addActionT(INTAKEWAIT3) { intake.status = SInvert }
            ts.at(put1Traj)

            val put2Traj = Trajectory(put2Pos)
            ts.at(put2Traj)

            val put3Traj = Trajectory(put3Pos)
            put3Traj.addActionE(70.0) { clown.goUp(-2); slides.setTarget(RMID_POS) }
            ts.at(put3Traj)
            ts.aa { clown.open() }
            ts.sl(WaitPut / 2.0)
        }
        val parkPos = sbParkPos.duplicate()
        parkPos.sp = sbTrajStack2Back[2].ep + sbPutOffset * (ncycle - 1)

        /// Put -> Park
        val parkTraj = Trajectory(parkPos)
        parkTraj.addActionS(0.0) { clown.open() }
        parkTraj.addActionS(50.0) { clown.goDown(); slides.setTarget(RBOT_POS) }
        ts.at(parkTraj)
        return ts
    }

    @JvmStatic
    fun getCycleTrajShortRed(ncycle: Int, randomCase: Int): TrajectorySequence {
        val preloadPos = srPPos[randomCase].duplicate()
        val preloadTraj = Trajectory(preloadPos)

        val putFromPreloadPos = srPutFromPreloadPos.duplicate()
        putFromPreloadPos.sp = preloadPos.ep
        putFromPreloadPos.ep.y += srPutYOffsetCase[randomCase]
        putFromPreloadPos.ep.x = srPutXCase[randomCase]
        val putFromPreloadTraj = Trajectory(putFromPreloadPos)


        val ts = TrajectorySequence()
        /// Start (DiffyPreloadUp) -> Go to preload (DiffyOpenFar)
        ts.aa { clown.targetPos = DiffyUpSafe }
        preloadTraj.addActionT(0.1) { clown.goPreloadUp() }
        preloadTraj.addActionS(60.0) { intake.status = SUpulLuiCostacu }
        ts.at(preloadTraj)
        ts.aa { clown.ghearaFar?.position = ClownFDeschis }
        /// Preload -> Put (DiffyUp)
        ts.aa { clown.targetPos = DiffyUpSafe }
        putFromPreloadTraj.addActionE(60.0) {
            clown.targetPos = DiffyUp
            clown.targetAngle = DiffyAUp
            clown.gelenk?.position = GelenkCenter + 2 * GelenkDif
        }
        ts.at(putFromPreloadTraj)
        ts.aa { clown.open() }
        //ts.sl(0.1)
        for (i in 0 until ncycle) {
            /// Put -> Inter1 (Diffy down) -> Inter2 -> Stack (Diffy down + gheara inchisa -> gheara deschisa + intake)
            val inter1Pos = srStackPPose[0].duplicate()
            inter1Pos.sp = putFromPreloadPos.ep
            inter1Pos.ep += srStackOffset * i
            inter1Pos.setNoEndBegin()
            val inter2Pos = srStackPPose[1].duplicate()
            inter2Pos.sp = inter1Pos.ep
            inter2Pos.ep += srStackOffset * i
            inter2Pos.setNoEndContinue()
            val inter3Pos = srStackPPose[2].duplicate()
            inter3Pos.sp = inter2Pos.ep
            inter3Pos.ep += srStackOffset * i
            inter3Pos.setNoEndContinue()
            val inter4Pos = srStackPPose[3].duplicate()
            inter4Pos.sp = inter3Pos.ep
            inter4Pos.ep += srStackOffset * i
            inter4Pos.setNoEndEnd()

            val put1Pos = srStackPPut[0].duplicate()
            put1Pos.sp = inter4Pos.ep
            put1Pos.ep += srPutOffset * i
            put1Pos.setNoEndBegin()
            val put2Pos = srStackPPut[1].duplicate()
            put2Pos.sp = put1Pos.ep
            put2Pos.ep += srPutOffset * i
            put2Pos.setNoEndContinue()
            val put3Pos = srStackPPut[2].duplicate()
            put3Pos.sp = put2Pos.ep
            put3Pos.ep += srPutOffset * i
            put3Pos.setNoEndEnd()

            val inter1Traj = Trajectory(inter1Pos)
            inter1Traj.addActionS(50.0) { intake.status = SIntake; clown.goDown(); slides.setTarget(RBOT_POS) }
            ts.at(inter1Traj)

            val inter2Traj = Trajectory(inter2Pos)
            ts.at(inter2Traj)

            val inter3Traj = Trajectory(inter3Pos)
            ts.at(inter3Traj)

            val inter4Traj = Trajectory(inter4Pos)
            if (i == 0) {
                inter4Traj.addActionE(80.0) { intake.status = SStack1 }
                ts.at(inter4Traj)
                ts.aa { intake.status = SStack2 }
                ts.sl(INTAKEWAIT1)
                ts.aa { intake.status = SStack3 }
                ts.sl(INTAKEWAIT2)
            } else {
                ts.at(inter4Traj)
                ts.sl(INTAKEWAIT4)
            }
            ts.aa { clown.catchPixel() }
            // val inter4Traj = Trajectory(inter4Pos)
            // ts.at(inter4Traj)

            /// Stack -> Inter2 -> Inter1 -> Put (Gheara inchisa -> Diffy up + gheara deschisa)
            val put1Traj = Trajectory(put1Pos)
            put1Traj.addActionT(INTAKEWAIT3) { intake.status = SInvert }
            ts.at(put1Traj)

            val put2Traj = Trajectory(put2Pos)
            ts.at(put2Traj)

            val put3Traj = Trajectory(put3Pos)
            put3Traj.addActionE(70.0) { clown.goUp(-2); slides.setTarget(RMID_POS) }
            ts.at(put3Traj)
            ts.aa { clown.open() }
            ts.sl(WaitPut / 2.0)
        }
        val parkPos = srParkPos.duplicate()
        parkPos.sp = srStackPPut[2].ep + srPutOffset * (ncycle - 1)

        /// Put -> Park
        val parkTraj = Trajectory(parkPos)
        parkTraj.addActionS(0.0) { clown.open() }
        parkTraj.addActionS(50.0) { clown.goDown(); slides.setTarget(RBOT_POS) }
        ts.at(parkTraj)
        return ts
    }


}
