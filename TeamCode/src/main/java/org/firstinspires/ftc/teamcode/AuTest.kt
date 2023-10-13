package org.firstinspires.ftc.teamcode

import com.acmerobotics.dashboard.config.Config
import com.qualcomm.robotcore.eventloop.opmode.Autonomous
import com.qualcomm.robotcore.eventloop.opmode.Disabled
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import org.firstinspires.ftc.teamcode.pp.Trajectory
import org.firstinspires.ftc.teamcode.utils.Pose
import org.firstinspires.ftc.teamcode.utils.RobotFuncs.endma
import org.firstinspires.ftc.teamcode.utils.RobotFuncs.initma
import org.firstinspires.ftc.teamcode.utils.RobotFuncs.log_state
import org.firstinspires.ftc.teamcode.utils.RobotFuncs.pp
import org.firstinspires.ftc.teamcode.utils.RobotFuncs.preinit
import org.firstinspires.ftc.teamcode.utils.RobotFuncs.startma
import org.firstinspires.ftc.teamcode.utils.Vec2d

@Disabled
@Autonomous(name = "我愛修訂")
class AuTest : LinearOpMode() {
    companion object {
        @JvmField
        var sp: Pose = Pose()
        @JvmField
        var ep: Pose = Pose()
        @JvmField
        var v1: Vec2d = Vec2d()
        @JvmField
        var v2: Vec2d = Vec2d()
        @JvmField
        var h1: Vec2d = Vec2d()

    }
    override fun runOpMode() {
        preinit()
        initma(this)

        waitForStart()

        startma()

        pp.startFollowTraj(Trajectory(sp, 0.0, ep, v1, v2, h1))
        while (!isStopRequested && !pp.done && !pp.error) {
            pp.update()
            log_state()
        }
        endma()
    }
}