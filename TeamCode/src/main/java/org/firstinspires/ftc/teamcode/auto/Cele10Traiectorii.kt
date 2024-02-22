package org.firstinspires.ftc.teamcode.auto

import com.qualcomm.robotcore.util.ElapsedTime
import org.firstinspires.ftc.teamcode.auto.AutoVars.WaitIntake
import org.firstinspires.ftc.teamcode.auto.AutoVars.WaitPut
import org.firstinspires.ftc.teamcode.auto.AutoVars.WaitStack
import org.firstinspires.ftc.teamcode.auto.AutoVars.colours
import org.firstinspires.ftc.teamcode.auto.BlueLongP.bPPos
import org.firstinspires.ftc.teamcode.auto.BlueLongP.bPStack
import org.firstinspires.ftc.teamcode.auto.BlueLongP.bParkPos
import org.firstinspires.ftc.teamcode.auto.BlueLongP.bPutOffset
import org.firstinspires.ftc.teamcode.auto.BlueLongP.bPutPos
import org.firstinspires.ftc.teamcode.auto.BlueLongP.bPutXCase
import org.firstinspires.ftc.teamcode.auto.BlueLongP.bPutYOffsetCase
import org.firstinspires.ftc.teamcode.auto.BlueLongP.bStackOffset
import org.firstinspires.ftc.teamcode.auto.BlueLongP.bStackPos2
import org.firstinspires.ftc.teamcode.auto.BlueShortP.sbPPos
import org.firstinspires.ftc.teamcode.auto.BlueShortP.sbParkPos
import org.firstinspires.ftc.teamcode.auto.BlueShortP.sbPutFromPreloadPos
import org.firstinspires.ftc.teamcode.auto.BlueShortP.sbPutXCase
import org.firstinspires.ftc.teamcode.auto.BlueShortP.sbPutYOffsetCase
import org.firstinspires.ftc.teamcode.auto.BlueShortP.stackPPose
import org.firstinspires.ftc.teamcode.auto.BlueShortP.stackPPut
import org.firstinspires.ftc.teamcode.hardware.Intakes
import org.firstinspires.ftc.teamcode.pp.Trajectory
import org.firstinspires.ftc.teamcode.utils.RobotFuncs.clown
import org.firstinspires.ftc.teamcode.utils.RobotFuncs.intake
import org.firstinspires.ftc.teamcode.utils.RobotFuncs.pp
import java.util.Vector
import kotlin.concurrent.thread

/// -x = Goto: x
/// 1 = Trajectory
/// 2 = Function
/// 3 = SetGoto
/// 4 = Conditional
// 10 = None
class TSE(val type: Int, val initActio: () -> Unit, val checkDone: () -> Boolean, val conditional: () -> Int, val traj: Trajectory?) {
    constructor(type: Int, initActio: () -> Unit, checkDone: () -> Boolean, traj: Trajectory?) : this(type, initActio, checkDone, {0}, traj) // Trajectory Sequence Element
    constructor(type: Int, initActio: () -> Unit, checkDone: () -> Boolean) : this(type, initActio, checkDone, {0}, null) // Trajectory Sequence Element
    constructor(type: Int, conditional: () -> Int) : this(type, {}, {true}, conditional, null)
    constructor(type: Int) : this(type, {}, {true}, {0}, null)
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

    fun addTrajectory(t: Trajectory) {
        steps.add(
                TSE(1,
                        { pp.startFollowTraj(t) },
                        { !pp.busy },
                        t
                ))
    }

    fun at(t: Trajectory) = addTrajectory(t)

    fun addAction(a: () -> Unit) {
        steps.add(
                TSE(2, a)
                { true }
        )
    }

    fun aa(a: () -> Unit) = addAction(a)

    fun sleep(s: Double) {
        steps.add(
                TSE(2, { stimer.reset() })
                { stimer.seconds() > s }
        )
    }

    fun sl(s: Double) = sleep(s)

    private var ls = 0
    private var e = TSE(10, {}, { true })

    fun reset() {
        ls = 0
        e = TSE(10, {}, { true })
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
            while (!this.update() && !Thread.currentThread().isInterrupted) {
                Thread.sleep(2)
            }
        }
        t.start()
        return t
    }

    fun addTSE(t: TSE) = steps.add(t)
    fun duplicate(): TrajectorySequence {
        val ct = TrajectorySequence()
        for (s in steps) {
            ct.addTSE(s)
        }
        return ct
    }
}

object Cele10Traiectorii {
    @JvmStatic
    fun getCycleTrajLongBlue(ncycle: Int, randomCase: Int): TrajectorySequence {
        val preloadPos = bPPos[randomCase].duplicate()
        val preloadTraj = Trajectory(preloadPos)

        val stackPos = bPStack[randomCase].duplicate()
        stackPos.sp = preloadPos.ep
        val stackTraj = Trajectory(stackPos)

        val putPos = bPutPos.duplicate()
        putPos.sp = stackPos.ep
        putPos.ep.y += bPutYOffsetCase[randomCase]
        putPos.ep.x = bPutXCase[randomCase]
        var putTraj = Trajectory(putPos)

        val stack2Pos = bStackPos2.duplicate()

        val parkPos = bParkPos.duplicate()
        parkPos.sp = putPos.ep
        val parkTraj = Trajectory(parkPos)

        val ts = TrajectorySequence()
        /// Start -> Go to preload (Spit out pixel)
        //preloadTraj.addActionE(20.0) { clown.position = GhearaSDESCHIS } /// TODO: Add intake
        ts.at(preloadTraj)
        ts.aa { intake.status = Intakes.SInvert }
        /// Preload -> Stack (Gheara deschisa + intake)
        ts.sl(WaitIntake)
        ts.aa { intake.status = Intakes.SUp; clown.goDown() }
        ts.at(stackTraj)
        ts.sl(WaitStack)
        /// Stack -> Put (Ridicare diffy + gheara deschisa)
        putTraj.addActionE(100.0) { intake.status = Intakes.SNothing; clown.goUp(0) }
        ts.at(putTraj)
        ts.sl(WaitPut)
        ts.aa { clown.open() }
        ts.sl(WaitPut)
        for (i in 0 until ncycle - 1) {
            /// Put -> Stack2 (Diffy down + gheara inchisa -> gheara deschisa + intake)
            stack2Pos.sp = putPos.ep
            stack2Pos.ep += bStackOffset * i
            val stack2Traj = Trajectory(stack2Pos)
            stack2Traj.addActionS(30.0) { clown.goDown() }
            stack2Traj.addActionE(50.0) {  } /// TODO: Add intake
            ts.at(stack2Traj)
            ts.sl(WaitStack)

            /// Stack2 -> Put (Gheara inchisa -> Diffy up + gheara deschisa)
            putPos.sp = stack2Pos.ep
            putPos.ep = putPos.ep + bPutOffset * i
            putTraj = Trajectory(putPos)
            putTraj.addActionE(130.0) { clown.goUp(0) }
            ts.at(putTraj)
            ts.sl(WaitPut)
            ts.aa { clown.open() }
            ts.sl(WaitPut)
        }
        /// Put -> Park
        parkTraj.addActionS(0.0) { clown.open() }
        parkTraj.addActionS(70.0) { clown.goDown() }
        ts.at(parkTraj)
        return ts
    }

    @JvmStatic
    fun getCycleTrajShortBlue(ncycle: Int, randomCase: Int): TrajectorySequence {
        val preloadPos = sbPPos[randomCase].duplicate()
        val preloadTraj = Trajectory(preloadPos)

        val putFromPreloadPos = sbPutFromPreloadPos.duplicate()
        putFromPreloadPos.sp = preloadPos.ep
        putFromPreloadPos.ep.y += sbPutYOffsetCase[randomCase]
        putFromPreloadPos.ep.x = sbPutXCase[randomCase]
        val putFromPreloadTraj = Trajectory(putFromPreloadPos)

        val inter1Pos = stackPPose[0].duplicate()
        inter1Pos.sp = putFromPreloadPos.ep
        inter1Pos.timeout = 0.0
        val inter2Pos = stackPPose[1].duplicate()
        inter2Pos.sp = inter1Pos.ep
        inter2Pos.timeout = 0.0
        inter2Pos.initVel = 10000.0
        val inter3Pos = stackPPose[2].duplicate()
        inter3Pos.sp = inter2Pos.ep
        inter3Pos.initVel = 10000.0

        val put1Pos = stackPPut[0].duplicate()
        put1Pos.sp = inter3Pos.ep
        put1Pos.timeout = 0.0
        val put2Pos = stackPPut[1].duplicate()
        put2Pos.sp = put1Pos.ep
        put2Pos.timeout = 0.0
        put2Pos.initVel = 10000.0
        val put3Pos = stackPPut[2].duplicate()
        put3Pos.sp = put2Pos.ep
        put3Pos.initVel = 10000.0


        val parkPos = sbParkPos.duplicate()
        parkPos.sp = put3Pos.ep

        val ts = TrajectorySequence()
        /// Start -> Go to preload (Spit out pixel)
        ts.at(preloadTraj)
        ts.aa { intake.status = Intakes.SInvert }
        ts.sl(WaitIntake)
        ts.aa { intake.status = Intakes.SUp }
        /// Preload -> Put (Gheara deschisa + intake)
        ts.aa { clown.goUp(0) }
        ts.at(putFromPreloadTraj)
        ts.sl(WaitPut)
        ts.aa { clown.open() }
        ts.sl(WaitPut / 2.0)
        for (i in 0 until ncycle) {
            /// Put -> Inter1 (Diffy down) -> Inter2 -> Stack (Diffy down + gheara inchisa -> gheara deschisa + intake)
            val inter1Traj = Trajectory(inter1Pos)
            inter1Traj.addActionS(30.0) { intake.status = Intakes.SNothing; clown.goDown() }
            ts.at(inter1Traj)

            val inter2Traj = Trajectory(inter2Pos)
            ts.at(inter2Traj)

            val inter3Traj = Trajectory(inter3Pos)
            inter3Traj.addActionE(50.0) { } /// TODO: Add intake
            ts.at(inter3Traj)
            ts.sl(WaitStack)

            /// Stack -> Inter2 -> Inter1 -> Put (Gheara inchisa -> Diffy up + gheara deschisa)
            val put1Traj = Trajectory(put1Pos)
            /// TODO: Stop intake
            ts.at(put1Traj)

            val put2Traj = Trajectory(put2Pos)
            ts.at(put2Traj)

            val put3Traj = Trajectory(put3Pos)
            put3Traj.addActionE(130.0) { clown.goUp(0) }
            ts.at(put3Traj)
            ts.sl(WaitPut)
            ts.aa { clown.open() }
            ts.sl(WaitPut / 2.0)
        }
        /// Put -> Park
        val parkTraj = Trajectory(parkPos)
        parkTraj.addActionS(0.0) { clown.open() }
        parkTraj.addActionS(70.0) { clown.goDown() }
        ts.at(parkTraj)
        return ts
    }

    @JvmStatic
    fun getCycleTrajShortRed(ncycle: Int, randomCase: Int): TrajectorySequence {
        return TrajectorySequence()
    }

    @JvmStatic
    fun getCycleTrajLongRed(ncycle: Int, randomCase: Int): TrajectorySequence {
        return TrajectorySequence()
    }

}
