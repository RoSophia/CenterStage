package org.firstinspires.ftc.teamcode

import com.acmerobotics.dashboard.config.Config
import com.outoftheboxrobotics.photoncore.Photon
import com.qualcomm.robotcore.eventloop.opmode.Autonomous
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import org.firstinspires.ftc.teamcode.AUTest.AAAAAAAAAAAAAAAAAAA
import org.firstinspires.ftc.teamcode.AUTest.AINt
import org.firstinspires.ftc.teamcode.AUTest.GO_TO_POS
import org.firstinspires.ftc.teamcode.AUTest.LOOP
import org.firstinspires.ftc.teamcode.AUTest.ep
import org.firstinspires.ftc.teamcode.AUTest.h1
import org.firstinspires.ftc.teamcode.AUTest.sp
import org.firstinspires.ftc.teamcode.AUTest.v1
import org.firstinspires.ftc.teamcode.AUTest.v2
import org.firstinspires.ftc.teamcode.pp.Trajectory
import org.firstinspires.ftc.teamcode.utils.Pose
import org.firstinspires.ftc.teamcode.utils.RobotFuncs.cam
import org.firstinspires.ftc.teamcode.utils.RobotFuncs.controller
import org.firstinspires.ftc.teamcode.utils.RobotFuncs.endma
import org.firstinspires.ftc.teamcode.utils.RobotFuncs.initAuto
import org.firstinspires.ftc.teamcode.utils.RobotFuncs.initma
import org.firstinspires.ftc.teamcode.utils.RobotFuncs.localizer
import org.firstinspires.ftc.teamcode.utils.RobotFuncs.log
import org.firstinspires.ftc.teamcode.utils.RobotFuncs.pp
import org.firstinspires.ftc.teamcode.utils.RobotFuncs.preinit
import org.firstinspires.ftc.teamcode.utils.RobotFuncs.startma
import org.firstinspires.ftc.teamcode.utils.RobotFuncs.update
import org.firstinspires.ftc.teamcode.utils.Vec2d

@Config
object AUTest {
    @JvmField
    var sp = Pose(0.0, 0.0, 0.0)

    @JvmField
    var ep = Pose(250.0, 70.0, 0.0)

    @JvmField
    var v1 = Vec2d(280.0, -0.1)

    @JvmField
    var v2 = Vec2d(50.0, -1.7)

    @JvmField
    var h1: Vec2d = Vec2d()

    @JvmField
    var AAAAAAAAAAAAAAAAAAA: Boolean = false

    @JvmField
    var AINt: Int = 0

    @JvmField
    var GO_TO_POS: Boolean = false

    @JvmField
    var LOOP: Boolean = true
}

@Photon
@Autonomous(name = "我愛修訂")
class AuTest : LinearOpMode() {
    override fun runOpMode() {
        preinit()
        initma(this)
        initAuto()
        waitForStart()
        cam.stop()
        startma()
        val t1 = Trajectory(sp, 0.0, ep, v1, v2, h1)
        val t2 = Trajectory(ep, 0.0, sp, v2, v1, h1)
        log("T1", t1)
        var at = 0

        var lep = Pose()

        while (!isStopRequested) {
            if (GO_TO_POS) {
                log("GO_TO", pp.ctraj)
                log("VALS", "$ep  $lep")
                if (ep.x != lep.x || ep.y != lep.y || lep.h != ep.h) {
                    log("GOT_DIF", "$ep != $lep")
                    pp.startFollowTraj(Trajectory(localizer.pose, localizer.poseVel.dist(), ep))
                    lep = ep.copy()
                }
                pp.update()
            } else {
                log("ppd", pp.done)
                if (LOOP) {
                    if (!pp.busy) {
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
                }
                controller.update()
                if (AAAAAAAAAAAAAAAAAAA) {
                    log("PPINT", pp.intersects(sp, AINt).toString())
                    pp.startFollowTraj(Trajectory(sp, 0.0, ep, v1, v2, h1))
                    pp.draw(pp.ctraj, Pose(), Pose(100000000000000.0, 0.0, 0.0), 0.0, 0.0, 0.0)
                } else {
                    pp.update()
                }
            }
            update()
        }

        endma()
    }
}