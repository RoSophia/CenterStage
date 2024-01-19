package org.firstinspires.ftc.teamcode.auto

import com.acmerobotics.dashboard.config.Config
import com.qualcomm.robotcore.util.ElapsedTime
import org.firstinspires.ftc.teamcode.Auto10Lini
import org.firstinspires.ftc.teamcode.auto.MKMKMKMKMKMKM.leftPos
import org.firstinspires.ftc.teamcode.auto.MKMKMKMKMKMKM.midPos
import org.firstinspires.ftc.teamcode.auto.MKMKMKMKMKMKM.parkPos
import org.firstinspires.ftc.teamcode.auto.MKMKMKMKMKMKM.putPos
import org.firstinspires.ftc.teamcode.auto.MKMKMKMKMKMKM.rightPos
import org.firstinspires.ftc.teamcode.auto.MKMKMKMKMKMKM.stackPos
import org.firstinspires.ftc.teamcode.hardware.Intakes
import org.firstinspires.ftc.teamcode.pp.TrajCoef
import org.firstinspires.ftc.teamcode.pp.Trajectory
import org.firstinspires.ftc.teamcode.utils.Pose
import org.firstinspires.ftc.teamcode.utils.RobotFuncs
import org.firstinspires.ftc.teamcode.utils.RobotFuncs.clown
import org.firstinspires.ftc.teamcode.utils.RobotFuncs.diffy
import org.firstinspires.ftc.teamcode.utils.RobotFuncs.intake
import org.firstinspires.ftc.teamcode.utils.RobotFuncs.pp
import org.firstinspires.ftc.teamcode.utils.RobotVars
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
            Pose(-82.0, 6.0, 1.57),
    )

    @JvmField
    var midPos = TrajCoef(
            Pose(-122.0, -11.0, 0.0),
            Vec2d(50.0, 2.0),
            Vec2d(25.0, 3.0)
    )

    @JvmField
    var rightPos = TrajCoef(
            Pose(-82.0, -13.0, -1.57)
    )

    @JvmField
    var stackPos = TrajCoef(
            Pose(-128.0, 55.0, 1.57)
    )

    @JvmField
    var putPos = TrajCoef(
            stackPos.ep,
            Pose(-54.0, -235.0, 1.57),
            Vec2d(230.0, 4.6),
            Vec2d(100.0, 2.34)
    )

    @JvmField
    var parkPos = TrajCoef(
            Pose(-80.0, -235.0, 1.57)
    )
}

class TrajectorySequence {
    val steps = Vector<TSE>()
    val stimer = ElapsedTime()

    fun draw() {
        for (s in steps) {
            if (s.type == 1) {
                if (s.traj != null) {
                    pp.drawTraj(s.traj)
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
                TSE(2,
                        a
                ) { true })
    }

    fun sleep(s: Double) {
        steps.add(
                TSE(2,
                        { stimer.reset() },
                        { stimer.seconds() > s }
                ))
    }

    var ls = 0
    var e = TSE(10, {}, { true })

    fun reset() {
        ls = 0
        e = TSE(10, {}, { true })
    }

    fun update() {
        if (ls < steps.size) {
            if (e.type == 10) {
                ls = 0
                e = steps[0]
                e.initActio()
            }
            while (e.checkDone()) {
                ++ls
                if (ls < steps.size) {
                    steps[ls].initActio()
                } else {
                    break
                }
            }
        }
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
        ts.sleep(0.4)
        ts.addAction { intake.status = Intakes.SNothing }

        stackPos.sp = preloadTraj.end
        val stackTraj = Trajectory(stackPos)
        stackTraj.addActionE(30.0) { intake.status = Intakes.SPStack1 }

        ts.addTrajectory(stackTraj)
        ts.addAction { intake.status = Intakes.SStack1 }
        ts.sleep(0.5)

        val putTraj = Trajectory(putPos)
        putTraj.addActionS(0.0) { intake.status = Intakes.SNothing; clown.position = RobotVars.GhearaSINCHIS }
        putTraj.addActionE(100.0) { diffy.targetPos = RobotVars.DiffyUp }

        ts.addTrajectory(putTraj)

        val stackTraj2 = Trajectory(stackPos)
        stackTraj2.addActionS(0.0) { clown.position = RobotVars.GhearaSDESCHIS }
        stackTraj2.addActionS(80.0) { clown.position = RobotVars.GhearaSINCHIS; diffy.targetPos = RobotVars.DiffyDown }
        stackTraj2.addActionS(0.0) { intake.status = Intakes.SNothing }
        stackTraj2.addActionE(30.0) { intake.status = Intakes.SPStack2 }
        stackTraj2.addActionE(0.0) { intake.status = Intakes.SStack2 }

        ts.addTrajectory(stackTraj2)
        ts.addAction { intake.status = Intakes.SNothing; clown.position = RobotVars.GhearaSINCHIS }

        val putTraj2 = Trajectory(putPos)
        putTraj2.addActionE(100.0) { diffy.targetPos = RobotVars.DiffyUp }
        putTraj2.addActionE(0.0) { clown.position = RobotVars.GhearaSDESCHIS }

        ts.addTrajectory(putTraj2)

        val goPark = Trajectory(parkPos)
        goPark.addActionS(0.0) { clown.position = RobotVars.GhearaSDESCHIS }
        goPark.addActionS(70.0) { diffy.targetPos = RobotVars.DiffyDown }
        ts.addTrajectory(goPark)

        return ts
    }
}
