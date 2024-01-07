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
    var ep: Pose = Pose(10.0, 10.0, 0.0)
    @JvmField
    var v1: Vec2d = Vec2d()
    @JvmField
    var v2: Vec2d = Vec2d()
    @JvmField
    var h1: Vec2d = Vec2d()
    @JvmField
    var AAAAAAAAAAAAAAAAAAA: Boolean = false
    @JvmField
    var AINt: Int = 0

    @JvmField
    var AUTO_MOVE: Boolean = false
}

@Photon
@Autonomous(name = "我愛修訂")
class AuTest : LinearOpMode() {
    override fun runOpMode() {
        preinit()
        initma(this)
        waitForStart()
        startma()
        pp.startFollowTraj(Trajectory(sp, 0.0, ep, v1, v2, h1))

        while (!isStopRequested && !pp.done && !pp.error) {
            controller.update()
            if (AAAAAAAAAAAAAAAAAAA) {
                log("PPINT", pp.intersects(sp, AINt).toString())
                val tp = TelemetryPacket()
                val canv = tp.fieldOverlay()
                canv.setStrokeWidth(10)
                canv.setStroke("#4CAF50")
                canv.strokeCircle(sp.x, sp.y, sp.h)

                FtcDashboard.getInstance().sendTelemetryPacket(tp)
            } else {
                pp.update()
                if (MOVE_SWERVE) {
                    moveSwerve()
                }
            }
            RobotFuncs.update()
        }

        endma()
    }
}