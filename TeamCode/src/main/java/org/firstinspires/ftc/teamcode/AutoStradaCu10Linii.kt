package org.firstinspires.ftc.teamcode

import com.acmerobotics.dashboard.config.Config
import com.outoftheboxrobotics.photoncore.Photon
import com.qualcomm.robotcore.eventloop.opmode.Autonomous
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import org.firstinspires.ftc.teamcode.Auto10Lini.justDraw
import org.firstinspires.ftc.teamcode.Auto10Lini.l1v
import org.firstinspires.ftc.teamcode.Auto10Lini.l2v
import org.firstinspires.ftc.teamcode.Auto10Lini.leftPos
import org.firstinspires.ftc.teamcode.Auto10Lini.lh
import org.firstinspires.ftc.teamcode.Auto10Lini.m1v
import org.firstinspires.ftc.teamcode.Auto10Lini.m2v
import org.firstinspires.ftc.teamcode.Auto10Lini.midPos
import org.firstinspires.ftc.teamcode.Auto10Lini.r1v
import org.firstinspires.ftc.teamcode.Auto10Lini.r2v
import org.firstinspires.ftc.teamcode.Auto10Lini.rh
import org.firstinspires.ftc.teamcode.Auto10Lini.rightPos
import org.firstinspires.ftc.teamcode.Auto10Lini.targetPos
import org.firstinspires.ftc.teamcode.pp.Trajectory
import org.firstinspires.ftc.teamcode.utils.Pose
import org.firstinspires.ftc.teamcode.utils.RobotFuncs.controller
import org.firstinspires.ftc.teamcode.utils.RobotFuncs.endma
import org.firstinspires.ftc.teamcode.utils.RobotFuncs.initma
import org.firstinspires.ftc.teamcode.utils.RobotFuncs.localizer
import org.firstinspires.ftc.teamcode.utils.RobotFuncs.log
import org.firstinspires.ftc.teamcode.utils.RobotFuncs.pp
import org.firstinspires.ftc.teamcode.utils.RobotFuncs.preinit
import org.firstinspires.ftc.teamcode.utils.RobotFuncs.startma
import org.firstinspires.ftc.teamcode.utils.RobotFuncs.update
import org.firstinspires.ftc.teamcode.utils.RobotVars.AutoInitPos
import org.firstinspires.ftc.teamcode.utils.Vec2d

@Config
object Auto10Lini {
    @JvmField var targetPos = 0
    @JvmField var justDraw = false

    @JvmField var leftPos = Pose(-82.0, 6.0, 1.57)
    @JvmField var lh = Vec2d(0.6, 0.95)
    @JvmField var l1v = Vec2d()
    @JvmField var l2v = Vec2d()

    @JvmField var midPos = Pose(-122.0, -22.0, 0.0)
    @JvmField var m1v = Vec2d(50.0, 2.0)
    @JvmField var m2v = Vec2d(25.0, 3.0)

    @JvmField var rightPos = Pose(-82.0, -13.0, -1.57)
    @JvmField var rh = Vec2d(0.6, 0.95)
    @JvmField var r1v = Vec2d(0.0, 0.0)
    @JvmField var r2v = Vec2d(0.0, 0.0)
}

@Photon
@Autonomous(name = "አውራ ጎዳና ከአስር መንገዶች ጋር")
class AutoStradaCu10Linii : LinearOpMode() {
    override fun runOpMode() {
        preinit()
        initma(this)
        waitForStart()
        startma()

        var ctraj: Trajectory
        ctraj = when (targetPos) {
            0 -> {
                Trajectory(AutoInitPos, 0.0, leftPos, l1v, l2v, lh)
            }
            1 -> {
                Trajectory(AutoInitPos, 0.0, midPos, m1v, m2v, Vec2d())
            }
            else -> {
                Trajectory(AutoInitPos, 0.0, rightPos, r1v, r2v, rh)
            }
        }
        pp.startFollowTraj(ctraj)

        while (!isStopRequested && !pp.done && !pp.error) {
            if (justDraw) {
                ctraj = when (targetPos) {
                    0 -> {
                        Trajectory(AutoInitPos, 0.0, leftPos, l1v, l2v, lh)
                    }
                    1 -> {
                        Trajectory(AutoInitPos, 0.0, midPos, m1v, m2v, Vec2d())
                    }
                    else -> {
                        Trajectory(AutoInitPos, 0.0, rightPos, r1v, r2v, rh)
                    }
                }
                pp.startFollowTraj(ctraj)
                pp.drawTraj(ctraj)
            } else {
                controller.update()
                pp.update()
            }
            update()
        }

        endma()
    }

}