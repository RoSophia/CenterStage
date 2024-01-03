package org.firstinspires.ftc.teamcode.pp

import com.acmerobotics.dashboard.FtcDashboard
import com.acmerobotics.dashboard.config.Config
import com.acmerobotics.dashboard.telemetry.TelemetryPacket
import org.firstinspires.ftc.teamcode.hardware.Swerve
import org.firstinspires.ftc.teamcode.utils.Pose
import org.firstinspires.ftc.teamcode.utils.RobotFuncs.dashboard
import org.firstinspires.ftc.teamcode.utils.Util.angDiff
import org.firstinspires.ftc.teamcode.utils.Util.eps
import kotlin.math.abs
import kotlin.math.atan
import kotlin.math.cos
import kotlin.math.sin
import kotlin.math.sqrt

class PurePursuit(private val swerve: Swerve, private val localizer: Localizer) {
    companion object {
        @JvmField
        var lookaheadRadius: Double = 1.0

        @JvmField
        var robotRadius: Double = 0.5

        @JvmField
        var lkr: Double = 1.0

        @JvmField
        var SCALE: Double = 0.6

        @JvmField
        var PV: Double = 0.6

        @JvmField
        var PH: Double = 0.6

        @JvmField
        var HAPPY_VEL: Double = 0.6

        @JvmField
        var HAPPY_DIST: Double = 0.6

        @JvmField
        var TSC: Double = 1.0

        @JvmField
        var SPC: Double = 1.0
    }

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

    fun intersects(p: Pose, i: Int): Pose? { // TODO: Got damn this code is an affront to god
        val sp = ctraj[i * ctraj.checkLen]
        val ed = ctraj[(i + 1) * ctraj.checkLen]
        val dp = ed - sp
        val l1 = Pose(dp.y / dp.x, -1.0, sp.y)
        val l2 = l1 * l1
        val l2xy = l2.x + l2.y
        val lr2 = lookaheadRadius * lookaheadRadius

        val x0 = -l1.x * l1.h / l2xy
        val y0 = -l1.y * l1.h / l2xy

        return if (l2.h > lr2 * l2xy + eps) {
            null
        } else if (abs(l2.h - lr2 * l2xy) < eps) {
            Pose(x0, y0, sp.h)
        } else {
            val d = lr2 - l2.h / l2xy
            val mult: Double = sqrt(d / l2xy)
            val ax: Double = x0 + l1.y * mult
            val bx: Double = x0 - l1.y * mult
            val ay: Double = y0 - l1.x * mult
            val by: Double = y0 + l1.x * mult
            closer(Pose(ax, ay, sp.h), Pose(bx, by, sp.h), ed)
        }
    }

    fun lookahead(cp: Pose): Pose { // TODO: Kill youserlf Now!
        var res: Pose? = null
        for (i in lastIndex..ctraj.checkpoints) {
            val ip = intersects(cp, i)
            if (ip != null) {
                res = ip
            } else if (res != null) {
                lastIndex = i - 1
                break
            }
        }
        if (res == null) {
            return ctraj[lastIndex * ctraj.checkLen]
        }
        return res
    }

    private fun draw(t: Trajectory, p: Pose, lk: Pose, speed: Double, angle: Double, tspeed: Double) {
        val tp = TelemetryPacket()
        val canvas = tp.fieldOverlay()
        canvas.setStrokeWidth(1)
        canvas.setStroke("#4CAF50")
        for (i in 0 until t.checkpoints) {
            canvas.strokeLine(t[i * t.checkLen].x * SCALE, t[i * t.checkLen].y * SCALE, t[(i + 1) * t.checkLen].x * SCALE, t[(i + 1) * t.checkLen].y * SCALE)
        }

        canvas.setStroke("#FF00C3")
        canvas.strokeCircle(p.x * SCALE, p.y * SCALE, robotRadius)
        canvas.setStroke("#FF00C3A0")
        canvas.strokeCircle(p.x * SCALE, p.y * SCALE, lookaheadRadius)
        canvas.setStroke("#FFFF00")
        canvas.fillCircle(lk.x * SCALE, lk.y * SCALE, lkr)
        canvas.setStroke("#40FF22")
        canvas.strokeLine(p.x * SCALE, p.y * SCALE, (p.x + speed * cos(angle) * SPC) * SCALE, (p.y + speed * sin(angle) * SPC) * SCALE)
        canvas.setStrokeWidth(3)
        canvas.setStroke("#78B00A0")
        canvas.strokeLine(100.0, 1.0, 100.0 + tspeed * TSC, 1.0)

        dashboard.sendTelemetryPacket(tp)
    }

    fun startFollowTraj(t: Trajectory) {
        ctraj = t
        lastIndex = 0
        done = false
        error = false
        haveTraj = true
    }

    fun gangle(o1: Pose, o2: Pose): Double {
        val o12 = o2 - o1
        return atan(o12.y / o12.x)
    }

    fun update() { /// TODO: Add bump detection
        if (busy) {
            val cp = localizer.pose
            val cv = localizer.poseVel

            val lk = lookahead(cp)

            val d = (lk - cp).dist() /// TODO: This code is just shit
            if ((ctraj.end - cp).dist() < HAPPY_DIST && cv.dist() < HAPPY_VEL) {
                done = true
            }
            val speed = ctraj.getSpeed(lastIndex * ctraj.checkLen).v2d().dist() + (d * PV)
            val ang = gangle(cp, lk)
            val tspeed = angDiff(ctraj[lastIndex * ctraj.checkLen].h, cp.h) * PH
            swerve.move(speed, ang, tspeed)
            draw(ctraj, cp, lk, speed, ang, tspeed)
        }
    }
}
