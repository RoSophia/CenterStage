package org.firstinspires.ftc.teamcode.auto

import com.qualcomm.robotcore.util.ElapsedTime
import org.firstinspires.ftc.teamcode.auto.AutoVars.SLEEPY_TIME
import org.firstinspires.ftc.teamcode.auto.AutoVars.WaitIntake
import org.firstinspires.ftc.teamcode.auto.AutoVars.WaitStack
import org.firstinspires.ftc.teamcode.auto.AutoVars.cmtime
import org.firstinspires.ftc.teamcode.auto.AutoVars.colours
import org.firstinspires.ftc.teamcode.auto.AutoVars.mtime
import org.firstinspires.ftc.teamcode.auto.AutoVars.putPosCase
import org.firstinspires.ftc.teamcode.auto.AutoVars.sputPosCase
import org.firstinspires.ftc.teamcode.auto.BlueLongP.bOffsets
import org.firstinspires.ftc.teamcode.auto.BlueLongP.bPos0
import org.firstinspires.ftc.teamcode.auto.BlueLongP.bPos0Stack
import org.firstinspires.ftc.teamcode.auto.BlueLongP.bPos1
import org.firstinspires.ftc.teamcode.auto.BlueLongP.bPos1Stack
import org.firstinspires.ftc.teamcode.auto.BlueLongP.bPos2
import org.firstinspires.ftc.teamcode.auto.BlueLongP.bPos2Stack
import org.firstinspires.ftc.teamcode.auto.BlueLongP.bparkPos
import org.firstinspires.ftc.teamcode.auto.BlueLongP.bPutPos
import org.firstinspires.ftc.teamcode.auto.BlueLongP.bStackOffset
import org.firstinspires.ftc.teamcode.auto.BlueLongP.bStackPos2
import org.firstinspires.ftc.teamcode.auto.BlueShortP.sbPos0
import org.firstinspires.ftc.teamcode.auto.BlueShortP.sbPos1
import org.firstinspires.ftc.teamcode.auto.BlueShortP.sbparkPos
import org.firstinspires.ftc.teamcode.auto.BlueShortP.sbPutPos
import org.firstinspires.ftc.teamcode.auto.BlueShortP.sbPos2
import org.firstinspires.ftc.teamcode.auto.BlueShortP.sbPos0Stack
import org.firstinspires.ftc.teamcode.auto.BlueShortP.sbPos1Stack
import org.firstinspires.ftc.teamcode.auto.BlueShortP.sbPos2Stack
import org.firstinspires.ftc.teamcode.auto.RedLongP.rOffsets
import org.firstinspires.ftc.teamcode.auto.RedLongP.rParkPos
import org.firstinspires.ftc.teamcode.auto.RedLongP.rPos0
import org.firstinspires.ftc.teamcode.auto.RedLongP.rPos0Stack
import org.firstinspires.ftc.teamcode.auto.RedLongP.rPos1
import org.firstinspires.ftc.teamcode.auto.RedLongP.rPos1Stack
import org.firstinspires.ftc.teamcode.auto.RedLongP.rPos2
import org.firstinspires.ftc.teamcode.auto.RedLongP.rPos2Stack
import org.firstinspires.ftc.teamcode.auto.RedLongP.rPutPos
import org.firstinspires.ftc.teamcode.auto.RedLongP.rPutPosCase
import org.firstinspires.ftc.teamcode.hardware.Intakes
import org.firstinspires.ftc.teamcode.pp.PP.MAX_TIME
import org.firstinspires.ftc.teamcode.pp.TrajCoef
import org.firstinspires.ftc.teamcode.pp.Trajectory
import org.firstinspires.ftc.teamcode.utils.RobotFuncs.clown
import org.firstinspires.ftc.teamcode.utils.RobotFuncs.diffy
import org.firstinspires.ftc.teamcode.utils.RobotFuncs.intake
import org.firstinspires.ftc.teamcode.utils.RobotFuncs.pp
import org.firstinspires.ftc.teamcode.utils.RobotFuncs.slides
import org.firstinspires.ftc.teamcode.utils.RobotVars.DiffyDown
import org.firstinspires.ftc.teamcode.utils.RobotVars.DiffyUp
import org.firstinspires.ftc.teamcode.utils.RobotVars.DiffyfDown
import org.firstinspires.ftc.teamcode.utils.RobotVars.DiffyfUp
import org.firstinspires.ftc.teamcode.utils.RobotVars.GhearaSDESCHIS
import org.firstinspires.ftc.teamcode.utils.RobotVars.GhearaSINCHIS
import org.firstinspires.ftc.teamcode.utils.RobotVars.RBOT_POS
import org.firstinspires.ftc.teamcode.utils.RobotVars.RMID_POS
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
        ts.addAction { intake.status = Intakes.SNothing }

        val stackPos: TrajCoef = when (randomCase) {
            2 -> bPos2Stack
            1 -> bPos1Stack
            else -> bPos0Stack
        }
        stackPos.sp = preloadPos.ep
        ts.addTrajectory(Trajectory(stackPos))

        val cp = bPutPos
        cp.sp = stackPos.ep
        cp.ep.y += when (randomCase) {
            2 -> bOffsets.h
            1 -> bOffsets.y
            else -> bOffsets.x
        }
        cp.ep.x = when (randomCase) {
            2 -> putPosCase.h
            1 -> putPosCase.y
            else -> putPosCase.x
        }
        val putTraj = Trajectory(cp)
        /*
        putTraj.addActionS(60.0) { intake.status = Intakes.SIntake }
        putTraj.addActionE(170.0) { clown.position = GhearaSINCHIS; intake.status = Intakes.SIntake }
        putTraj.addActionE(150.0) { clown.position = GhearaSDESCHIS }
        putTraj.addActionE(120.0) { clown.position = GhearaSINCHIS; intake.status = Intakes.SIntake }*/
        putTraj.addActionE(100.0) { slides.setTarget(RMID_POS / 2, 0.0) }
        putTraj.addActionE(100.0) { intake.status = Intakes.SNothing; diffy.targetPos = DiffyUp; diffy.targetDiff = DiffyfDown }
        ts.addTrajectory(putTraj)
        ts.sleep(1.0)
        ts.addAction { clown.position = GhearaSDESCHIS }
        ts.sleep(1.0)

        for (i in 0 until ncycle - 1) {
            bStackPos2.sp = putTraj.end
            var cs = bStackPos2.duplicate()
            cs.sp += bStackOffset * i.toDouble()
            cs.ep += bStackOffset * i.toDouble()
            val stackTraj2 = Trajectory(cs)
            stackTraj2.addActionS(55.0) { clown.position = GhearaSINCHIS; diffy.targetPos = DiffyDown; diffy.targetDiff = DiffyfUp; slides.setTarget(RBOT_POS, 0.0) }
            stackTraj2.addActionS(0.0) { intake.status = Intakes.SNothing }
            stackTraj2.addActionE(30.0) { clown.position = GhearaSDESCHIS; intake.status = if (i == 0) Intakes.SPStack2 else Intakes.SPStack3 }

            ts.addTrajectory(stackTraj2)
            ts.addAction { intake.status = if (i == 0) Intakes.SStack2 else Intakes.SStack3 }
            ts.sleep(WaitStack)

            cs = bPutPos.duplicate()
            cs.sp += bStackOffset * i.toDouble()
            cs.ep.x = putPosCase.y
            cs.ep += bStackOffset * i.toDouble()
            val putTraj2 = Trajectory(cs)
            putTraj2.addActionS(100.0) { intake.status = Intakes.SIntake }
            putTraj2.addActionE(200.0) { clown.position = GhearaSINCHIS }
            putTraj2.addActionE(150.0) { slides.setTarget(RMID_POS / 2, 0.0) }
            putTraj2.addActionE(100.0) { intake.status = Intakes.SNothing; diffy.targetPos = DiffyUp }
            ts.addTrajectory(putTraj2)
            ts.addAction { clown.position = GhearaSDESCHIS }
        }

        bparkPos.sp = putTraj.end
        val goPark = Trajectory(bparkPos)
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
        putTraj.addActionE(100.0) { slides.setTarget(RMID_POS / 2, 0.0) }
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
        ts.addAction { intake.status = Intakes.SNothing }

        val stackPos: TrajCoef = when (randomCase) {
            2 -> rPos2Stack
            1 -> rPos1Stack
            else -> rPos0Stack
        }
        stackPos.sp = preloadPos.ep
        ts.addTrajectory(Trajectory(stackPos))
        ts.sleep(SLEEPY_TIME)

        val cp = rPutPos
        cp.sp = stackPos.ep
        cp.ep.y += when (randomCase) {
            2 -> rOffsets.h
            1 -> rOffsets.y
            else -> rOffsets.x
        }
        cp.ep.x = when (randomCase) {
            2 -> rPutPosCase.h
            1 -> rPutPosCase.y
            else -> rPutPosCase.x
        }
        val putTraj = Trajectory(cp)

        putTraj.addActionE(100.0) { slides.setTarget(RMID_POS / 2, 0.0) }
        putTraj.addActionE(100.0) { intake.status = Intakes.SNothing; diffy.targetPos = DiffyUp; diffy.targetDiff = DiffyfDown }
        ts.addTrajectory(putTraj)
        ts.sleep(1.0)
        ts.addAction { clown.position = GhearaSDESCHIS }
        ts.sleep(1.0)

        rParkPos.sp = putTraj.end
        val goPark = Trajectory(rParkPos)
        goPark.addActionS(0.0) { clown.position = GhearaSDESCHIS }
        goPark.addActionS(70.0) { clown.position = GhearaSINCHIS; diffy.targetPos = DiffyDown; diffy.targetDiff = DiffyfUp }
        ts.addTrajectory(goPark)

        return ts
    }
}
