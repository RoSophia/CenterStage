package org.firstinspires.ftc.teamcode.auto

import com.acmerobotics.dashboard.config.Config
import com.qualcomm.robotcore.util.ElapsedTime
import org.firstinspires.ftc.teamcode.auto.MKMKMKMKMKMKM.WaitIntake
import org.firstinspires.ftc.teamcode.auto.MKMKMKMKMKMKM.WaitStack
import org.firstinspires.ftc.teamcode.auto.MKMKMKMKMKMKM.colours
import org.firstinspires.ftc.teamcode.auto.MKMKMKMKMKMKM.leftPos
import org.firstinspires.ftc.teamcode.auto.MKMKMKMKMKMKM.midPos
import org.firstinspires.ftc.teamcode.auto.MKMKMKMKMKMKM.parkPos
import org.firstinspires.ftc.teamcode.auto.MKMKMKMKMKMKM.putPos
import org.firstinspires.ftc.teamcode.auto.MKMKMKMKMKMKM.rightPos
import org.firstinspires.ftc.teamcode.auto.MKMKMKMKMKMKM.stackOffset
import org.firstinspires.ftc.teamcode.auto.MKMKMKMKMKMKM.stackPos
import org.firstinspires.ftc.teamcode.auto.MKMKMKMKMKMKM.stackPos2
import org.firstinspires.ftc.teamcode.hardware.Intakes
import org.firstinspires.ftc.teamcode.pp.PP.MAX_FRACTION
import org.firstinspires.ftc.teamcode.pp.TrajCoef
import org.firstinspires.ftc.teamcode.pp.Trajectory
import org.firstinspires.ftc.teamcode.utils.Pose
import org.firstinspires.ftc.teamcode.utils.RobotFuncs.clown
import org.firstinspires.ftc.teamcode.utils.RobotFuncs.diffy
import org.firstinspires.ftc.teamcode.utils.RobotFuncs.intake
import org.firstinspires.ftc.teamcode.utils.RobotFuncs.pp
import org.firstinspires.ftc.teamcode.utils.RobotFuncs.slides
import org.firstinspires.ftc.teamcode.utils.RobotVars.*
import org.firstinspires.ftc.teamcode.utils.Vec2d
import java.util.Vector

/// 1 = Trajectory
/// 2 = Function
// 10 = None
class TSE(val type: Int, val initActio: () -> Unit, val checkDone: () -> Boolean, val traj: Trajectory?) {
    constructor(type: Int, initActio: () -> Unit, checkDone: () -> Boolean) : this(type, initActio, checkDone, null)
// Trajectory Sequence Element
}

@Config
object MKMKMKMKMKMKM {
    @JvmField
    var leftPos = TrajCoef(
            Pose(-105.0, 10.0, 0.5),
            Vec2d(0.0, 0.0),
            0.45
    )

    @JvmField
    var midPos = TrajCoef(
            Pose(-122.0, -11.0, 0.0),
            Vec2d(50.0, 2.0),
            Vec2d(25.0, 3.0),
            Vec2d(),
            0.5
    )

    @JvmField
    var rightPos = TrajCoef(
            Pose(-90.0, -13.0, -1.57),
            0.5
    )

    @JvmField
    var stackPos = TrajCoef(
            Pose(-131.0, 71.0, 1.57),
            Vec2d(40.0, 4.0), Vec2d(40.0, 4.0),
            0.5
    )

    @JvmField
    var putPos = TrajCoef(
            stackPos.ep,
            Pose(-65.0, -220.0, 1.57),
            Vec2d(200.0, 4.3),
            Vec2d(80.0, 2.34),
            Vec2d(), MAX_FRACTION, Vec2d(60.0, 120.0)
    )

    @JvmField
    var stackPos2 = TrajCoef(
            putPos.ep,
            Pose(-130.0, 69.0, 1.57),
            Vec2d(100.0, -3.5),
            Vec2d(100.0, -1.35),
            Vec2d(), MAX_FRACTION, Vec2d(60.0, 100.0)
    )

    @JvmField
    var parkPos = TrajCoef(
            Pose(-110.0, -210.0, 1.57)
    )

    @JvmField
    var stackOffset = Pose(0.0, 3.5, 0.0)

    @JvmField
    var NumCycles = 3

    @JvmField
    var WaitIntake = 0.3

    @JvmField
    var WaitStack = 0.4

    val colours = arrayOf("#254E70", "#37718E", "#8EE3EF", "#AEF3E7", "#F6BD60", "#F7EDE2", "#F5CAC3", "#84A59D", "#F28482", "#19535F", "#0B7A75", "#D7C9AA", "#7B2D26", "#F0F3F5")
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

class Cele10Traiectorii {
    fun getCycleTraj(ncycle: Int, isRed: Boolean, randomCase: Int): TrajectorySequence {
        val ang = if (isRed) -1.0 else 1.0
        val ts = TrajectorySequence()
        val preloadTraj: Trajectory = when (randomCase) {
            0 -> {
                Trajectory(leftPos)
            }

            1 -> {
                Trajectory(midPos)
            }

            else -> {
                Trajectory(rightPos)
            }
        }
        ts.addTrajectory(preloadTraj)
        ts.addAction { intake.status = Intakes.SInvert }
        ts.sleep(WaitIntake)

        stackPos.sp = preloadTraj.end
        val stackTraj = Trajectory(stackPos)
        stackTraj.addActionE(40.0) { intake.status = Intakes.SPStack1 }

        ts.addTrajectory(stackTraj)
        ts.addAction { clown.position = GhearaSDESCHIS; intake.status = Intakes.SStack1 }
        ts.sleep(WaitStack)

        val putTraj = Trajectory(putPos)
        putTraj.addActionS(100.0) { intake.status = Intakes.SIntake }
        putTraj.addActionE(200.0) { clown.position = GhearaSINCHIS }
        putTraj.addActionE(150.0) { slides.setTarget(RMID_POS / 2, 0.0) }
        putTraj.addActionE(100.0) { intake.status = Intakes.SNothing; diffy.targetPos = DiffyUp }
        ts.addTrajectory(putTraj)
        ts.addAction { clown.position = GhearaSDESCHIS }

        for (i in 0 until ncycle - 1) {
            stackPos2.sp = putTraj.end
            var cs = stackPos2.duplicate()
            cs.sp += stackOffset * i.toDouble()
            cs.ep += stackOffset * i.toDouble()
            val stackTraj2 = Trajectory(cs)
            stackTraj2.addActionS(80.0) { clown.position = GhearaSINCHIS; diffy.targetPos = DiffyDown; slides.setTarget(RBOT_POS, 0.0) }
            stackTraj2.addActionS(0.0) { intake.status = Intakes.SNothing }
            stackTraj2.addActionE(30.0) { clown.position = GhearaSDESCHIS; intake.status = if (i == 0) Intakes.SPStack2 else Intakes.SPStack3 }

            ts.addTrajectory(stackTraj2)
            ts.addAction { intake.status = if (i == 0) Intakes.SStack2 else Intakes.SStack3 }
            ts.sleep(WaitStack)

            cs = putPos.duplicate()
            cs.sp += stackOffset * i.toDouble()
            cs.ep += stackOffset * i.toDouble()
            val putTraj2 = Trajectory(cs)
            putTraj2.addActionS(100.0) { intake.status = Intakes.SIntake }
            putTraj2.addActionE(200.0) { clown.position = GhearaSINCHIS }
            putTraj2.addActionE(150.0) { slides.setTarget(RMID_POS / 2, 0.0) }
            putTraj2.addActionE(100.0) { intake.status = Intakes.SNothing; diffy.targetPos = DiffyUp }
            ts.addTrajectory(putTraj2)
            ts.addAction { clown.position = GhearaSDESCHIS }
        }

        stackPos2.sp = putTraj.end
        val cs = stackPos2.duplicate()
        cs.ep += Pose(0.0, -40.0, 0.0)
        val stackTraj2 = Trajectory(cs)
        stackTraj2.addActionE(100.0) { diffy.targetPos = DiffyDown }
        ts.addTrajectory(stackTraj2)

        /*
        parkPos.sp = putPos.ep
        val goPark = Trajectory(parkPos)
        goPark.addActionS(0.0) { clown.position = GhearaSDESCHIS }
        goPark.addActionS(70.0) { diffy.targetPos = DiffyDown }
        ts.addTrajectory(goPark)
         */

        return ts
    }
}
