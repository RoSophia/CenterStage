package org.firstinspires.ftc.teamcode.pp

import com.acmerobotics.dashboard.config.Config
import com.acmerobotics.dashboard.telemetry.TelemetryPacket
import com.qualcomm.robotcore.util.ElapsedTime
import org.firstinspires.ftc.teamcode.AUTest.AUTO_MOVE
import org.firstinspires.ftc.teamcode.AUTest.sp
import org.firstinspires.ftc.teamcode.hardware.PIDFC
import org.firstinspires.ftc.teamcode.hardware.Swerve
import org.firstinspires.ftc.teamcode.pp.DriveConstants.MAX_ACC
import org.firstinspires.ftc.teamcode.pp.DriveConstants.MAX_DEC
import org.firstinspires.ftc.teamcode.pp.DriveConstants.MAX_FRACTION
import org.firstinspires.ftc.teamcode.pp.DriveConstants.MAX_VEL
import org.firstinspires.ftc.teamcode.pp.PP.HAPPY_DIST
import org.firstinspires.ftc.teamcode.pp.PP.HAPPY_HEAD
import org.firstinspires.ftc.teamcode.pp.PP.HAPPY_HEAD_VEL
import org.firstinspires.ftc.teamcode.pp.PP.HAPPY_VEL
import org.firstinspires.ftc.teamcode.pp.PP.HeadingPid
import org.firstinspires.ftc.teamcode.pp.PP.PERU_COEF
import org.firstinspires.ftc.teamcode.pp.PP.PositionPid
import org.firstinspires.ftc.teamcode.pp.PP.SCALE
import org.firstinspires.ftc.teamcode.pp.PP.SPC
import org.firstinspires.ftc.teamcode.pp.PP.TSC
import org.firstinspires.ftc.teamcode.pp.PP.lkr
import org.firstinspires.ftc.teamcode.pp.PP.lookaheadRadius
import org.firstinspires.ftc.teamcode.pp.PP.robotRadius
import org.firstinspires.ftc.teamcode.utils.Pose
import org.firstinspires.ftc.teamcode.utils.RobotFuncs
import org.firstinspires.ftc.teamcode.utils.RobotFuncs.dashboard
import org.firstinspires.ftc.teamcode.utils.RobotFuncs.log
import org.firstinspires.ftc.teamcode.utils.RobotFuncs.logs
import org.firstinspires.ftc.teamcode.utils.RobotFuncs.moveSwerve
import org.firstinspires.ftc.teamcode.utils.RobotFuncs.timmy
import org.firstinspires.ftc.teamcode.utils.RobotVars.MOVE_SWERVE
import org.firstinspires.ftc.teamcode.utils.Util.angDiff
import org.firstinspires.ftc.teamcode.utils.Util.angNorm
import kotlin.math.abs
import kotlin.math.atan
import kotlin.math.cos
import kotlin.math.max
import kotlin.math.min
import kotlin.math.sign
import kotlin.math.sin

@Config
object PP {
    @JvmField
    var lookaheadRadius: Double = 10.0

    @JvmField
    var robotRadius: Double = 10.0

    @JvmField
    var lkr: Double = 2.0

    @JvmField
    var SCALE: Double = 0.2

    @JvmField
    var PositionPid: PIDFC = PIDFC(0.3, 0.0, 0.0, 0.0)

    @JvmField
    var HeadingPid: PIDFC = PIDFC(1.7, 0.0, 0.0, 0.0)

    @JvmField
    var HAPPY_VEL: Double = 0.3

    @JvmField
    var HAPPY_DIST: Double = 1.4

    @JvmField
    var HAPPY_HEAD: Double = 0.08

    @JvmField
    var HAPPY_HEAD_VEL: Double = 0.02

    @JvmField
    var PERU_COEF: Double = 1 / 30.0

    @JvmField
    var TSC: Double = 1.0

    @JvmField
    var SPC: Double = 200.0
}

class PurePursuit(private val swerve: Swerve, private val localizer: Localizer) {
    var ctraj = Trajectory()
    var lastIndex = 0
    var haveTraj = false // Only doing this since making ctraj null would be a lot of hassle
    var done = true
    var error = false

    val busy: Boolean
        get() {
            return haveTraj && !done && !error
        }

    fun closer(o1: Pose, o2: Pose, ed: Pose): Pose {
        val o11 = ed - o1
        val o22 = ed - o2
        return if (o11.x * o11.x + o11.y + o11.y < o22.x * o22.x + o22.y + o22.y) {
            o1
        } else {
            o2
        }
    }

    fun intersects(cp: Pose, i: Int): Pose? {
        val tp = ctraj[i * ctraj.checkLen]
        return if ((cp - tp).dist2() < lookaheadRadius * lookaheadRadius) {
            tp
        } else {
            null
        }
    }

    fun lookahead(cp: Pose): Pose { // TODO: Kill youserlf Now!
        var res: Pose? = null
        logs("Lookahead_1", cp)
        for (i in lastIndex..ctraj.checkpoints) {
            val ip = intersects(cp, i)
            if (ip != null) {
                res = ip
                lastIndex = i
            } else {
                break
            }
        }
        logs("Lookahead_lasti", lastIndex)
        logs("Lookahead_res", res.toString())
        if (res == null) {
            return ctraj[lastIndex * ctraj.checkLen]
        }
        return res
    }

    fun draw(t: Trajectory, p: Pose, lk: Pose, speed: Double, angle: Double, tspeed: Double) {
        val canvas = RobotFuncs.tp.fieldOverlay()
        canvas.setStrokeWidth(2)
        canvas.setStroke("#4CAF50")
        for (i in 0 until t.checkpoints step t.checkpoints / 50) {
            canvas.strokeLine(t[i * t.checkLen].x * SCALE, t[i * t.checkLen].y * SCALE, t[(i + t.checkpoints / 50) * t.checkLen].x * SCALE, t[(i + t.checkpoints / 50) * t.checkLen].y * SCALE)
        }

        canvas.setStrokeWidth(1)
        canvas.setStroke("#FF00C3A0")
        canvas.strokeCircle(p.x * SCALE, p.y * SCALE, lookaheadRadius * SCALE)
        canvas.setStroke("#3010FF")
        canvas.strokeLine(p.x * SCALE, p.y * SCALE,
                (p.x * SCALE + speed * cos(angle) * SPC), (p.y * SCALE + speed * sin(angle) * SPC))
        canvas.setStrokeWidth(3)
        canvas.setStroke("#000000")
        canvas.fillCircle(lk.x * SCALE, lk.y * SCALE, lkr)
        canvas.setStrokeWidth(1)
        canvas.strokeLine(10.0, 10.0, 10.0 + tspeed * TSC * SCALE, 10.0)
    }

    fun startFollowTraj(t: Trajectory) {
        ctraj = t
        lastIndex = 0
        done = false
        error = false
        haveTraj = true
        ep.reset()
        lperu = 0.0
        peruI = 0.0
        runningTime.reset()
    }

    fun gangle(o1: Pose, o2: Pose): Double {
        val o12 = o2 - o1
        return if (o12.x >= 0.0) {
            atan(o12.y / o12.x) - timmy.yaw
        } else {
            atan(o12.y / o12.x) + Math.PI - timmy.yaw
        }
    }

    var lperu = 0.0
    var lang = 0.0
    var peruI = 0.0
    var angI = 0.0
    var ep = ElapsedTime()
    var runningTime = ElapsedTime()
    fun update() { /// TODO: Add bump detection
        if (busy) {
            val cp = localizer.pose
            val cv = localizer.poseVel

            val lk = lookahead(cp)
            //val lk = Pose()
            logs("S_lk", lk)
            log("S_Dist", lk - cp)

            val peru = (cp - lk).dist() /// TODO: This code is just shit
            val peruD = (peru - lperu) / ep.seconds()
            lperu = peru
            peruI += peru * ep.seconds()
            logs("S_d", peru)
            if (abs(timmy.yawVel) < HAPPY_HEAD_VEL && abs(angDiff(ctraj.end.h, cp.h)) < HAPPY_HEAD && (ctraj.end - cp).dist() < HAPPY_DIST && cv.dist() < HAPPY_VEL) {
                swerve.move(0.0, swerve.angle, 0.0)
                done = true
                return
            }

            logs("S_ctraje", ctraj.end)
            //var speed = ctraj.getSpeed(lastIndex * ctraj.checkLen) + (peru * PositionPid.p + peruD * PositionPid.d + peruI * PositionPid.i + sign(peru) * PositionPid.f)

            val tcoef = min(MAX_VEL, min(ctraj.initVel + MAX_ACC * runningTime.seconds(), (ctraj.end - cp).dist() * PERU_COEF * MAX_DEC)) * MAX_FRACTION / MAX_VEL
            //val tcoef = min(MAX_VEL, ctraj.initVel + MAX_ACC * runningTime.seconds()) * MAX_FRACTION / MAX_VEL
            val pspeed = min(max(peru * PositionPid.p + peruD * PositionPid.d + peruI * PositionPid.i + sign(peru) * PositionPid.f, -1.0), 1.0)
            var speed = tcoef * pspeed
            log("SWERVE_Tcoef", tcoef)
            log("SWERVE_pspeed", pspeed)
            if (speed.isNaN()) {
                speed = 0.0
            }
            var ang = gangle(cp, lk)
            peruI += peru * ep.seconds()
            if (ang.isNaN()) {
                ang = 0.0
            }
            val ange = angDiff(cp.h, ctraj[lastIndex * ctraj.checkLen].h)
            val angD = angDiff(ange, lang) / ep.seconds()
            lang = ang
            angI += ange * ep.seconds()
            val tspeed = ange * HeadingPid.p + angD * HeadingPid.d + angI * HeadingPid.i + sign(ange) * HeadingPid.f

            if (AUTO_MOVE) {
                swerve.move(speed, ang, tspeed)
            } else if (MOVE_SWERVE) {
                moveSwerve()
            }
            draw(ctraj, cp, lk, speed, ang, ange)
            ep.reset()
        }
    }
}
