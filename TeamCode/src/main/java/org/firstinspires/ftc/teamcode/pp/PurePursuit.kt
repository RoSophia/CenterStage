package org.firstinspires.ftc.teamcode.pp

import com.acmerobotics.dashboard.config.Config
import com.acmerobotics.dashboard.telemetry.TelemetryPacket
import com.qualcomm.robotcore.util.ElapsedTime
import org.firstinspires.ftc.teamcode.AUTest.sp
import org.firstinspires.ftc.teamcode.hardware.PIDFC
import org.firstinspires.ftc.teamcode.hardware.Swerve
import org.firstinspires.ftc.teamcode.pp.PP.HAPPY_DIST
import org.firstinspires.ftc.teamcode.pp.PP.HAPPY_VEL
import org.firstinspires.ftc.teamcode.pp.PP.HeadingPid
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
import org.firstinspires.ftc.teamcode.utils.Util.angDiff
import kotlin.math.atan
import kotlin.math.cos
import kotlin.math.sign
import kotlin.math.sin

@Config
object PP {
    @JvmField
    var lookaheadRadius: Double = 10.0

    @JvmField
    var robotRadius: Double = 0.5

    @JvmField
    var lkr: Double = 15.0

    @JvmField
    var SCALE: Double = 0.2

    @JvmField
    var PositionPid: PIDFC = PIDFC(0.0, 0.0, 0.0, 0.0)

    @JvmField
    var HeadingPid: PIDFC = PIDFC(0.0, 0.0, 0.0, 0.0)

    @JvmField
    var HAPPY_VEL: Double = 0.3

    @JvmField
    var HAPPY_DIST: Double = 1.4

    @JvmField
    var TSC: Double = 1.0

    @JvmField
    var SPC: Double = 1.0
}

class PurePursuit(private val swerve: Swerve, private val localizer: Localizer) {
    var ctraj = Trajectory()
    var lastIndex = 0
    var haveTraj = false // Only doing this since making ctraj null would be a lot of hassle
    var done = false
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
        log("Lookahead_1", cp)
        for (i in lastIndex..ctraj.checkpoints) {
            val ip = intersects(cp, i)
            if (ip != null) {
                res = ip
                lastIndex = i - 1
            } else {
                break
            }
        }
        log("Lookahead_lasti", lastIndex)
        log("Lookahead_res", res.toString())
        if (res == null) {
            return ctraj[lastIndex * ctraj.checkLen]
        }
        return res
    }

    private fun draw(t: Trajectory, p: Pose, lk: Pose, speed: Double, angle: Double, tspeed: Double) {
        val canvas = RobotFuncs.tp.fieldOverlay()
        canvas.setStrokeWidth(2)
        canvas.setStroke("#4CAF50")
        for (i in 0 until t.checkpoints step t.checkpoints / 50) {
            canvas.strokeLine(t[i * t.checkLen].x * SCALE, t[i * t.checkLen].y * SCALE, t[(i + t.checkpoints / 50) * t.checkLen].x * SCALE, t[(i + t.checkpoints / 50) * t.checkLen].y * SCALE)
        }

        canvas.setStroke("#FF00C3")
        canvas.strokeCircle(p.x * SCALE, p.y * SCALE, robotRadius * SCALE)
        canvas.setStroke("#FF00C3A0")
        canvas.strokeCircle(p.x * SCALE, p.y * SCALE, lookaheadRadius * SCALE)
        canvas.setStroke("#FFFF00")
        canvas.fillCircle(lk.x * SCALE, lk.y * SCALE, lkr * SCALE)
        canvas.setStroke("#40FF22")
        canvas.strokeLine(p.x * SCALE, p.y * SCALE, (p.x + speed * cos(angle) * SPC) * SCALE, (p.y + speed * sin(angle) * SPC) * SCALE)
        canvas.setStrokeWidth(3)
        canvas.setStroke("#78B00A0")
        canvas.strokeLine(10.0, 1.0, 10.0 + tspeed * TSC * SCALE, 1.0)
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
    }

    fun gangle(o1: Pose, o2: Pose): Double {
        val o12 = o2 - o1
        return atan(o12.y / o12.x)
    }

    var lperu = 0.0
    var lang = 0.0
    var peruI = 0.0
    var angI = 0.0
    var ep = ElapsedTime()
    fun update() { /// TODO: Add bump detection
        if (busy) {
            val cp = localizer.pose
            val cv = localizer.poseVel
            log("S_Cp", cp)
            log("S_Cv", cv)

            val lk = lookahead(cp)
            //val lk = Pose()
            log("S_lk", lk)

            val peru = (lk - cp).dist() /// TODO: This code is just shit
            val peruD = (peru - lperu) / ep.seconds()
            lperu = peru
            peruI += peru * ep.seconds()
            log("S_d", peru)
            if ((ctraj.end - cp).dist() < HAPPY_DIST && cv.dist() < HAPPY_VEL) {
                done = true
            }

            log("S_ctraje", ctraj.end)
            log("S_spee", ctraj.getSpeed(lastIndex * ctraj.checkLen))
            var speed = ctraj.getSpeed(lastIndex * ctraj.checkLen).v2d().dist() + (peru * PositionPid.p + peruD * PositionPid.d + peruI * PositionPid.i + sign(peru) * PositionPid.f)
            if (speed.isNaN()) {
                speed = 0.0
            }
            var ang = gangle(cp, lk)
            peruI += peru * ep.seconds()
            if (ang.isNaN()) {
                ang = 0.0
            }
            val ange = angDiff(ctraj[lastIndex * ctraj.checkLen].h, cp.h)
            val angD = angDiff(ange, lang) / ep.seconds()
            lang = ang
            angI += ange * ep.seconds()
            val tspeed = ange * HeadingPid.p + angD * HeadingPid.d + angI * HeadingPid.i + sign(ange) * HeadingPid.f
            log("SWERVE_MOVE_S", speed)
            log("SWERVE_MOVE_A", ang)
            log("SWERVE_MOVE_tspeed", ang)

            swerve.move(speed, ang, tspeed)
            draw(ctraj, cp, lk, speed, ang, ange)
            ep.reset()
        }
    }
}
