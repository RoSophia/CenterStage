package org.firstinspires.ftc.teamcode.auto

import com.acmerobotics.dashboard.config.Config
import com.outoftheboxrobotics.photoncore.Photon
import com.qualcomm.robotcore.eventloop.opmode.Autonomous
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import org.firstinspires.ftc.teamcode.auto.AUTest.SLEPY
import org.firstinspires.ftc.teamcode.auto.AUTest.ep
import org.firstinspires.ftc.teamcode.auto.AUTest.h1
import org.firstinspires.ftc.teamcode.auto.AUTest.sp
import org.firstinspires.ftc.teamcode.auto.AUTest.v1
import org.firstinspires.ftc.teamcode.auto.AUTest.v2
import org.firstinspires.ftc.teamcode.auto.AutoFuncs.totBordu
import org.firstinspires.ftc.teamcode.auto.AutoFuncs.updateAutoCustom
import org.firstinspires.ftc.teamcode.pp.Trajectory
import org.firstinspires.ftc.teamcode.utils.Pose
import org.firstinspires.ftc.teamcode.utils.RobotFuncs.endma
import org.firstinspires.ftc.teamcode.utils.RobotFuncs.initma
import org.firstinspires.ftc.teamcode.utils.RobotFuncs.pp
import org.firstinspires.ftc.teamcode.utils.RobotFuncs.preinit
import org.firstinspires.ftc.teamcode.utils.RobotFuncs.startma
import org.firstinspires.ftc.teamcode.utils.Vec2d

@Config
object AUTest {
    @JvmField
    var sp = Pose(0.0, 0.0, 0.0)

    @JvmField
    var ep = Pose(200.0, 00.0, 0.0)

    @JvmField
    var v1 = Vec2d(000.0, -0.1)

    @JvmField
    var v2 = Vec2d(00.0, -1.7)

    @JvmField
    var h1: Vec2d = Vec2d()

    @JvmField
    var SLEPY = 0.5
}

@Photon
@Autonomous(name = "我愛修訂")
class AuTest : LinearOpMode() {
    override fun runOpMode() {
        totBordu(this, false)

        val updateTraj = {
            val cc = TrajectorySequence()
            cc.addTrajectory(Trajectory(sp, 0.0, ep, v1, v2, h1))
            cc.sleep(SLEPY)
            cc.addTrajectory(Trajectory(ep, 0.0, sp, v2, v1, h1))
            cc.sleep(SLEPY)
            cc
        }
        var ce = updateTraj()

        while (!isStopRequested) {
            val cce = updateAutoCustom(ce, updateTraj)
            if (cce != null) { ce = cce }
        }

        endma()
    }
}