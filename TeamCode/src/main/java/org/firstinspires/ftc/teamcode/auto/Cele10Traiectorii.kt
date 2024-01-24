package org.firstinspires.ftc.teamcode.auto

import com.qualcomm.robotcore.util.ElapsedTime
import org.firstinspires.ftc.teamcode.auto.AutoVars.WaitIntake
import org.firstinspires.ftc.teamcode.auto.AutoVars.cmtime
import org.firstinspires.ftc.teamcode.auto.AutoVars.colours
import org.firstinspires.ftc.teamcode.auto.AutoVars.mtime
import org.firstinspires.ftc.teamcode.auto.AutoVars.sputPosCase
import org.firstinspires.ftc.teamcode.auto.BlueLongP.bOffsets
import org.firstinspires.ftc.teamcode.auto.BlueLongP.bPos0
import org.firstinspires.ftc.teamcode.auto.BlueLongP.bPos0Stack
import org.firstinspires.ftc.teamcode.auto.BlueLongP.bPos1
import org.firstinspires.ftc.teamcode.auto.BlueLongP.bPos1Stack
import org.firstinspires.ftc.teamcode.auto.BlueLongP.bPos2
import org.firstinspires.ftc.teamcode.auto.BlueLongP.bPos2Stack
import org.firstinspires.ftc.teamcode.auto.BlueLongP.bPutPos
import org.firstinspires.ftc.teamcode.auto.BlueLongP.bPutPosCase
import org.firstinspires.ftc.teamcode.auto.BlueLongP.bparkPos
import org.firstinspires.ftc.teamcode.auto.BlueShortP.sbPos0
import org.firstinspires.ftc.teamcode.auto.BlueShortP.sbPos0Stack
import org.firstinspires.ftc.teamcode.auto.BlueShortP.sbPos1
import org.firstinspires.ftc.teamcode.auto.BlueShortP.sbPos1Stack
import org.firstinspires.ftc.teamcode.auto.BlueShortP.sbPos2
import org.firstinspires.ftc.teamcode.auto.BlueShortP.sbPos2Stack
import org.firstinspires.ftc.teamcode.auto.BlueShortP.sbPutPos
import org.firstinspires.ftc.teamcode.auto.BlueShortP.sbparkPos
import org.firstinspires.ftc.teamcode.auto.RedLongP.rOffsets
import org.firstinspires.ftc.teamcode.auto.RedLongP.rPos0
import org.firstinspires.ftc.teamcode.auto.RedLongP.rPos0Stack
import org.firstinspires.ftc.teamcode.auto.RedLongP.rPos1
import org.firstinspires.ftc.teamcode.auto.RedLongP.rPos1Stack
import org.firstinspires.ftc.teamcode.auto.RedLongP.rPos2
import org.firstinspires.ftc.teamcode.auto.RedLongP.rPos2Stack
import org.firstinspires.ftc.teamcode.auto.RedLongP.rPutPos
import org.firstinspires.ftc.teamcode.auto.RedLongP.rPutPosCase
import org.firstinspires.ftc.teamcode.auto.RedLongP.rparkPos
import org.firstinspires.ftc.teamcode.hardware.Intakes
import org.firstinspires.ftc.teamcode.pp.PP.MAX_TIME
import org.firstinspires.ftc.teamcode.pp.TrajCoef
import org.firstinspires.ftc.teamcode.pp.Trajectory
import org.firstinspires.ftc.teamcode.utils.RobotFuncs.clown
import org.firstinspires.ftc.teamcode.utils.RobotFuncs.diffy
import org.firstinspires.ftc.teamcode.utils.RobotFuncs.intake
import org.firstinspires.ftc.teamcode.utils.RobotFuncs.pp
import org.firstinspires.ftc.teamcode.utils.RobotVars.DiffyDown
import org.firstinspires.ftc.teamcode.utils.RobotVars.DiffyUp
import org.firstinspires.ftc.teamcode.utils.RobotVars.DiffyfDown
import org.firstinspires.ftc.teamcode.utils.RobotVars.DiffyfUp
import org.firstinspires.ftc.teamcode.utils.RobotVars.GhearaSDESCHIS
import org.firstinspires.ftc.teamcode.utils.RobotVars.GhearaSINCHIS
import java.util.Vector

/// 1 = Trajectory
/// 2 = Function
// 10 = None
class TSE(val type: Int, val initActio: () -> Unit, val checkDone: () -> Boolean, val traj: Trajectory?) {
    constructor(type: Int, initActio: () -> Unit, checkDone: () -> Boolean) : this(type, initActio, checkDone, null) // Trajectory Sequence Element
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

    fun addAction(a: () -> Unit) {
        steps.add(
                TSE(2, a)
                { true }
        )
    }

    fun sleep(s: Double) {
        steps.add(
                TSE(2, { stimer.reset() })
                { stimer.seconds() > s }
        )
    }

    private var ls = 0
    private var e = TSE(10, {}, { true })

    fun reset() {
        ls = 0
        e = TSE(10, {}, { true })
    }

    fun update(): Boolean {
        if (ls < steps.size) {
            if (e.type == 10) {
                ls = 0
                e = steps[0]
                e.initActio()
            }
            while (e.checkDone()) {
                ++ls
                if (ls < steps.size) {
                    e = steps[ls]
                    e.initActio()
                } else {
                    return true
                }
            }
        } else {
            return true
        }
        return false
    }
}

object Cele10Traiectorii {
    @JvmStatic
    fun getCycleTrajLongBlue(ncycle: Int, randomCase: Int): TrajectorySequence {
        val ts = TrajectorySequence()
        ts.addAction { MAX_TIME = cmtime }
        val preloadPos: TrajCoef = when (randomCase) {
            2 -> bPos2
            1 -> bPos1
            else -> bPos0
        }
        ts.addTrajectory(Trajectory(preloadPos))
        ts.addAction { MAX_TIME = mtime }
        ts.addAction { intake.status = Intakes.SInvert }
        ts.sleep(WaitIntake)
        ts.addAction { intake.status = Intakes.SUp }

        val stackPos: TrajCoef = when (randomCase) {
            2 -> bPos2Stack
            1 -> bPos1Stack
            else -> bPos0Stack
        }
        stackPos.sp = preloadPos.ep
        val stackTraj = Trajectory(stackPos)
        ts.addTrajectory(stackTraj)

        val cp = bPutPos.duplicate()
        cp.sp = stackPos.ep
        cp.ep.y += bOffsets[randomCase]
        cp.ep.x = bPutPosCase[randomCase]
        val putTraj = Trajectory(cp)
        putTraj.addActionE(100.0) { intake.status = Intakes.SNothing; diffy.targetPos = DiffyUp; diffy.targetDiff = DiffyfDown }
        ts.addTrajectory(putTraj)
        ts.sleep(0.2)
        ts.addAction { clown.position = GhearaSDESCHIS }
        ts.sleep(0.2)

        bparkPos.sp = putTraj.end
        val goPark = Trajectory(bparkPos)
        goPark.addActionS(0.0) { clown.position = GhearaSDESCHIS }
        goPark.addActionS(70.0) { clown.position = GhearaSINCHIS; diffy.targetPos = DiffyDown; diffy.targetDiff = DiffyfUp }
        ts.addTrajectory(goPark)

        return ts
    }

    @JvmStatic
    fun getCycleTrajLongRed(ncycle: Int, randomCase: Int): TrajectorySequence {
        val ts = TrajectorySequence()
        ts.addAction { MAX_TIME = cmtime }
        val preloadPos: TrajCoef = when (randomCase) {
            2 -> rPos2
            1 -> rPos1
            else -> rPos0
        }
        ts.addTrajectory(Trajectory(preloadPos))
        ts.addAction { MAX_TIME = mtime }
        ts.addAction { intake.status = Intakes.SInvert }
        ts.sleep(WaitIntake)
        ts.addAction { intake.status = Intakes.SUp }

        val stackPos: TrajCoef = when (randomCase) {
            2 -> rPos2Stack
            1 -> rPos1Stack
            else -> rPos0Stack
        }
        stackPos.sp = preloadPos.ep
        val stackTraj = Trajectory(stackPos)
        ts.addTrajectory(stackTraj)

        val cp = rPutPos.duplicate()
        cp.sp = stackPos.ep
        cp.ep.y += rOffsets[randomCase]
        cp.ep.x = rPutPosCase[randomCase]
        val putTraj = Trajectory(cp)
        putTraj.addActionE(100.0) { intake.status = Intakes.SNothing; diffy.targetPos = DiffyUp; diffy.targetDiff = DiffyfDown }
        ts.addTrajectory(putTraj)
        ts.sleep(0.2)
        ts.addAction { clown.position = GhearaSDESCHIS }
        ts.sleep(0.2)

        rparkPos.sp = putTraj.end
        val goPark = Trajectory(rparkPos)
        goPark.addActionS(0.0) { clown.position = GhearaSDESCHIS }
        goPark.addActionS(70.0) { clown.position = GhearaSINCHIS; diffy.targetPos = DiffyDown; diffy.targetDiff = DiffyfUp }
        ts.addTrajectory(goPark)
        return ts
    }

    @JvmStatic
    fun getCycleTrajShortBlue(ncycle: Int, randomCase: Int): TrajectorySequence {
        val ts = TrajectorySequence()
        ts.addAction { MAX_TIME = cmtime }
        val preloadTraj: Trajectory = when (randomCase) {
            2 -> Trajectory(sbPos2)
            1 -> Trajectory(sbPos1)
            else -> Trajectory(sbPos0)
        }
        ts.addTrajectory(preloadTraj)
        ts.addAction { MAX_TIME = mtime }
        ts.addAction { intake.status = Intakes.SInvert }
        ts.sleep(WaitIntake)
        ts.addAction { intake.status = Intakes.SNothing }

        val stackPos: TrajCoef = when (randomCase) {
            2 -> sbPos2Stack
            1 -> sbPos1Stack
            else -> sbPos0Stack
        }
        stackPos.sp = preloadTraj.end
        ts.addTrajectory(Trajectory(stackPos))

        val cp = sbPutPos.duplicate()
        cp.ep.x = when (randomCase) {
            2 -> sputPosCase.x
            1 -> sputPosCase.y
            else -> sputPosCase.h
        }
        val putTraj = Trajectory(cp)
        //putTraj.addActionE(100.0) { slides.setTarget(RMID_POS / 2, 0.0) }
        putTraj.addActionE(100.0) { intake.status = Intakes.SNothing; diffy.targetPos = DiffyUp; diffy.targetDiff = DiffyfDown }
        ts.addTrajectory(putTraj)
        ts.sleep(1.0)
        ts.addAction { clown.position = GhearaSDESCHIS }
        ts.sleep(1.0)

        sbparkPos.sp = sbPutPos.ep
        val goPark = Trajectory(sbparkPos)
        goPark.addActionS(0.0) { clown.position = GhearaSDESCHIS }
        goPark.addActionS(70.0) { clown.position = GhearaSINCHIS; diffy.targetPos = DiffyDown; diffy.targetDiff = DiffyfUp }
        ts.addTrajectory(goPark)

        return ts
    }

}
