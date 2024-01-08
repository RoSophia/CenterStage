package org.firstinspires.ftc.teamcode

import com.acmerobotics.dashboard.FtcDashboard
import com.acmerobotics.dashboard.config.Config
import com.acmerobotics.dashboard.telemetry.TelemetryPacket
import com.outoftheboxrobotics.photoncore.Photon
import com.qualcomm.robotcore.eventloop.opmode.Autonomous
import com.qualcomm.robotcore.eventloop.opmode.Disabled
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import com.qualcomm.robotcore.eventloop.opmode.OpMode
import com.qualcomm.robotcore.util.ElapsedTime
import org.firstinspires.ftc.teamcode.AUTest.AAAAAAAAAAAAAAAAAAA
import org.firstinspires.ftc.teamcode.AUTest.AINt
import org.firstinspires.ftc.teamcode.AUTest.AUTO_MOVE
import org.firstinspires.ftc.teamcode.AUTest.ep
import org.firstinspires.ftc.teamcode.AUTest.h1
import org.firstinspires.ftc.teamcode.AUTest.sp
import org.firstinspires.ftc.teamcode.AUTest.v1
import org.firstinspires.ftc.teamcode.AUTest.v2
import org.firstinspires.ftc.teamcode.pp.Trajectory
import org.firstinspires.ftc.teamcode.utils.Pose
import org.firstinspires.ftc.teamcode.utils.RobotFuncs
import org.firstinspires.ftc.teamcode.utils.RobotFuncs.KILLALL
import org.firstinspires.ftc.teamcode.utils.RobotFuncs.controller
import org.firstinspires.ftc.teamcode.utils.RobotFuncs.endma
import org.firstinspires.ftc.teamcode.utils.RobotFuncs.etime
import org.firstinspires.ftc.teamcode.utils.RobotFuncs.initma
import org.firstinspires.ftc.teamcode.utils.RobotFuncs.log
import org.firstinspires.ftc.teamcode.utils.RobotFuncs.log_state
import org.firstinspires.ftc.teamcode.utils.RobotFuncs.moveSwerve
import org.firstinspires.ftc.teamcode.utils.RobotFuncs.pp
import org.firstinspires.ftc.teamcode.utils.RobotFuncs.preinit
import org.firstinspires.ftc.teamcode.utils.RobotFuncs.send_log
import org.firstinspires.ftc.teamcode.utils.RobotFuncs.startma
import org.firstinspires.ftc.teamcode.utils.RobotVars.MOVE_SWERVE
import org.firstinspires.ftc.teamcode.utils.Vec2d

@Config
object AUTest {
    @JvmField
    var sp: Pose = Pose()

    @JvmField
    var ep: Pose = Pose(130.0, 20.0, 0.0)

    @JvmField
    var v1: Vec2d = Vec2d(130.0, -0.8)

    @JvmField
    var v2: Vec2d = Vec2d(80.0, 1.3)

    @JvmField
    var h1: Vec2d = Vec2d()

    @JvmField
    var AAAAAAAAAAAAAAAAAAA: Boolean = false

    @JvmField
    var AINt: Int = 0

    @JvmField
    var AUTO_MOVE: Boolean = true
}

@Photon
@Autonomous(name = "我愛修訂")
class AuTest : LinearOpMode() {
    override fun runOpMode() {
        preinit()
        initma(this)
        waitForStart()
        startma()
        val t1 = Trajectory(sp, 0.0, ep, v1, v2, h1)
        val t2 = Trajectory(ep, 0.0, sp, v2, v1, h1)
        var at = 0

        while (!isStopRequested) {
            log("ppd", pp.done)
            if (pp.done) {
                if (at == 0) {
                    log("st1", "")
                    at = 1
                    pp.startFollowTraj(t1)
                } else {
                    log("st2", "")
                    at = 0
                    pp.startFollowTraj(t2)
                }
            }
            controller.update()
            if (AAAAAAAAAAAAAAAAAAA) {
                log("PPINT", pp.intersects(sp, AINt).toString())
                pp.startFollowTraj(Trajectory(sp, 0.0, ep, v1, v2, h1))
                pp.draw(pp.ctraj, Pose(), Pose(100000000000000.0, 0.0, 0.0), 0.0, 0.0, 0.0)
            } else {
                pp.update()
            }
            RobotFuncs.update()
        }

        endma()
    }
}