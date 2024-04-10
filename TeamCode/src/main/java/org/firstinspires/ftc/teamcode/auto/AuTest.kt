package org.firstinspires.ftc.teamcode.auto

import com.acmerobotics.dashboard.config.Config
import com.outoftheboxrobotics.photoncore.Photon
import com.qualcomm.robotcore.eventloop.opmode.Autonomous
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import org.firstinspires.ftc.teamcode.auto.AUTest.SLEPY
import org.firstinspires.ftc.teamcode.auto.AUTest.ep
import org.firstinspires.ftc.teamcode.auto.AUTest.h1
import org.firstinspires.ftc.teamcode.auto.AUTest.peru
import org.firstinspires.ftc.teamcode.auto.AUTest.sp
import org.firstinspires.ftc.teamcode.auto.AUTest.v1
import org.firstinspires.ftc.teamcode.auto.AUTest.v2
import org.firstinspires.ftc.teamcode.pp.Trajectory
import org.firstinspires.ftc.teamcode.utils.Pose
import org.firstinspires.ftc.teamcode.utils.RobotFuncs.endma
import org.firstinspires.ftc.teamcode.utils.RobotFuncs.initma
import org.firstinspires.ftc.teamcode.utils.RobotFuncs.pp
import org.firstinspires.ftc.teamcode.utils.RobotFuncs.preinit
import org.firstinspires.ftc.teamcode.utils.RobotFuncs.startma
import org.firstinspires.ftc.teamcode.utils.RobotFuncs.update
import org.firstinspires.ftc.teamcode.utils.TrajectorySequence
import org.firstinspires.ftc.teamcode.utils.Vec2d

@Config
object AUTest {
    @JvmField
    var sp = Pose(0.0, 0.0, 0.0)

    @JvmField
    var ep = Pose(200.0, 100.0, 0.0)

    @JvmField
    var v1 = Vec2d(100.0, -0.1)

    @JvmField
    var v2 = Vec2d(80.0, -1.4)

    @JvmField
    var peru = Vec2d(30.0, 40.0)

    @JvmField
    var h1: Vec2d = Vec2d()

    @JvmField
    var SLEPY = 0.5
}

@Photon
@Autonomous(name = "我愛修訂", group = "ZZZZZZZZZZ")
class AuTest : LinearOpMode() {
    override fun runOpMode() {
        //totBordu(this, isRed = false, isShort = false)
        preinit()
        initma(this, false)
        waitForStart()
        startma()

        val updateTraj = {
            val cc = TrajectorySequence()
                    .st(1)
                    .at(Trajectory(sp, 0.0, ep, v1, v2, h1, 0.9, peru.x, peru.y, 2.0))
                    .sl(SLEPY)
                    .at(Trajectory(ep, 0.0, sp, v2, v1, h1, 0.9, peru.x, peru.y, 2.0))
                    .sl(SLEPY)
                    .gt{1}
            cc
        }
        var ce = updateTraj()

        while (!isStopRequested && !ce.update()) {
            pp.update()
            update()
        }

            /*
        while (!isStopRequested) {
            val cce = updateAutoCustom(ce, updateTraj)
            if (cce != null) {
                ce = cce
            }
        }
             */

        endma()
    }
}