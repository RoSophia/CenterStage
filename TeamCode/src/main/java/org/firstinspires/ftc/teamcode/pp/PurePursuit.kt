package org.firstinspires.ftc.teamcode.pp

import com.acmerobotics.dashboard.config.Config
import com.qualcomm.robotcore.util.ElapsedTime
import org.firstinspires.ftc.teamcode.hardware.PIDFC
import org.firstinspires.ftc.teamcode.hardware.Swerve
import org.firstinspires.ftc.teamcode.pp.PP.AR
import org.firstinspires.ftc.teamcode.pp.PP.PidAngle
import org.firstinspires.ftc.teamcode.pp.PP.Checkpoints
import org.firstinspires.ftc.teamcode.pp.PP.PidFinalLong
import org.firstinspires.ftc.teamcode.pp.PP.PidFinalTrans
import org.firstinspires.ftc.teamcode.pp.PP.HAPPY_DIST
import org.firstinspires.ftc.teamcode.pp.PP.HAPPY_HEAD
import org.firstinspires.ftc.teamcode.pp.PP.HAPPY_HEAD_VEL
import org.firstinspires.ftc.teamcode.pp.PP.HAPPY_VEL
import org.firstinspires.ftc.teamcode.pp.PP.PidLong
import org.firstinspires.ftc.teamcode.pp.PP.LookaheadScale
import org.firstinspires.ftc.teamcode.pp.PP.MAX_ACC
import org.firstinspires.ftc.teamcode.pp.PP.MAX_VEL
import org.firstinspires.ftc.teamcode.pp.PP.PPMaxAngPower
import org.firstinspires.ftc.teamcode.pp.PP.PPMaxSpeed
import org.firstinspires.ftc.teamcode.pp.PP.PPPPPPP
import org.firstinspires.ftc.teamcode.pp.PP.PPStartEnd
import org.firstinspires.ftc.teamcode.pp.PP.PPStaticSpeed
import org.firstinspires.ftc.teamcode.pp.PP.PeruMin
import org.firstinspires.ftc.teamcode.pp.PP.PeruMax
import org.firstinspires.ftc.teamcode.pp.PP.SCALE
import org.firstinspires.ftc.teamcode.pp.PP.SPC
import org.firstinspires.ftc.teamcode.pp.PP.TSC
import org.firstinspires.ftc.teamcode.pp.PP.PidTrans
import org.firstinspires.ftc.teamcode.pp.PP.lkr
import org.firstinspires.ftc.teamcode.utils.PID
import org.firstinspires.ftc.teamcode.utils.Pose
import org.firstinspires.ftc.teamcode.utils.RobotFuncs
import org.firstinspires.ftc.teamcode.utils.RobotFuncs.drawRobot
import org.firstinspires.ftc.teamcode.utils.RobotFuncs.ep
import org.firstinspires.ftc.teamcode.utils.RobotFuncs.log
import org.firstinspires.ftc.teamcode.utils.RobotFuncs.logs
import org.firstinspires.ftc.teamcode.utils.RobotFuncs.moveSwerve
import org.firstinspires.ftc.teamcode.utils.RobotVars.*
import org.firstinspires.ftc.teamcode.utils.Util.angDiff
import org.firstinspires.ftc.teamcode.utils.Util.clamp
import org.firstinspires.ftc.teamcode.utils.Util.epsEq
import org.firstinspires.ftc.teamcode.utils.Vec2d
import kotlin.math.abs
import kotlin.math.atan
import kotlin.math.cos
import kotlin.math.max
import kotlin.math.min
import kotlin.math.sin
import kotlin.math.sqrt

@Config
object PP {
    @JvmField
    var JustDraw = false

    @JvmField
    var robotRadius: Double = 10.0

    @JvmField
    var lkr: Double = 2.0

    @JvmField
    var SCALE: Double = 0.2

    @JvmField
    var HAPPY_VEL: Double = 0.3

    @JvmField
    var HAPPY_DIST: Double = 3.0

    @JvmField
    var HAPPY_HEAD: Double = 0.12

    @JvmField
    var HAPPY_HEAD_VEL: Double = 0.02

    @JvmField
    var MAX_TIME: Double = 2.3

    @JvmField
    var TSC: Double = 1.0

    @JvmField
    var SPC: Double = 200.0

    @JvmField
    var PPPPPPP: Double = 20.0

    @JvmField
    var AR: Double = 1.5

    @JvmField
    var MAX_ACC = 40.0

    @JvmField
    var MAX_VEL = 20.0

    @JvmField
    var MAX_FRACTION = 1.0

    @JvmField
    var PeruStart: Double = 10.0

    @JvmField
    var PeruEnd: Double = 40.0

    @JvmField
    var PeruMin: Double = 0.20

    @JvmField
    var PeruMax: Double = 1.0

    @JvmField
    var PidTrans = PIDFC(0.4, 0.0, 0.0, 0.0) // Trans rights

    @JvmField
    var PidLong = PIDFC(0.4, 0.0, 0.0, 0.6)

    @JvmField
    var PidFinalTrans = PIDFC(0.1, 0.0, 0.0, 0.4)

    @JvmField
    var PidFinalLong = PIDFC(0.1, 0.0, 0.0, 0.4)

    @JvmField
    var PidAngle = PIDFC(0.4, 0.0, 0.0, 0.00)

    @JvmField
    var PPStartEnd: Double = 15.0

    @JvmField
    var PPMaxAngPower: Double = 0.5

    @JvmField
    var PPMaxSpeed: Double = 1.0

    @JvmField
    var PPStaticSpeed: Double = 0.09

    @JvmField
    var Checkpoints: Int = 2000

    @JvmField
    var LookaheadScale: Vec2d = Vec2d(50.0, 30.0)
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

    fun scaleSep(v: Vec2d, i: Int, s: Vec2d): Vec2d {
        val trajAngle = gangle(ctraj.deriv(i).vec())
        val vR = v.rotated(-trajAngle)
        val vCR = Vec2d(vR.x * s.x, vR.y * s.y)
        return vCR.rotated(trajAngle)
    }

    fun intersects(cp: Pose, i: Int): Boolean {
        val ang = gangle(ctraj.deriv(i).vec())
        val vR = (cp - ctraj[i]).vec().rotated(-ang)
        val vRy = (abs(vR.x) < LookaheadScale.x && abs(vR.y) < LookaheadScale.y)
        val vAR = (cp - ctraj[i]).vec().rotated(ang)
        val vARy = (abs(vAR.x) < LookaheadScale.x && abs(vAR.y) < LookaheadScale.y)
        return vRy && vARy
    }

    private fun lookahead(cp: Pose): Pose {
        var lb = lastIndex
        var ub = Checkpoints
        var res = lastIndex

        while (lb < ub) {
            val c = (ub + lb) / 2
            if (intersects(cp, c)) {
                res = c
                lb = c + 1
            } else {
                ub = c - 1
            }
        }

        if (res == Checkpoints - 1) {
            res = Checkpoints
        }

        var ca = ctraj.nextAction()
        while (ca.checkNr <= res) {
            ca.act()
            ++ctraj.lastCompletedAction
            ca = ctraj.nextAction()
        }

        lastIndex = res
        return ctraj[res]
    }

    fun drawTraj(t: Trajectory, tcol: String) = draw(t, InfPos, InfPos, 0.0, 0.0, 0.0, tcol)
    fun drawTraj(t: Trajectory) = draw(t, InfPos, InfPos, 0.0, 0.0, 0.0, "#D7C9AA")

    fun draw(t: Trajectory, p: Pose, lk: Pose, speed: Double, angle: Double, tspeed: Double) = draw(t, p, lk, speed, angle, tspeed, "#D7C9AA")
    fun draw(t: Trajectory, p: Pose, lk: Pose, speed: Double, angle: Double, tspeed: Double, tcol: String) {
        val canvas = RobotFuncs.tp.fieldOverlay()
        canvas.setStrokeWidth(2)
        canvas.setStroke(tcol)
        var cc = 0
        for (i in 0 until Checkpoints step Checkpoints / 50) {
            if ((t.h.x <= (i * t.checkLen)) && ((i * t.checkLen) <= t.h.y)) {
                if (cc == 0) {
                    cc = 1
                    canvas.setStrokeWidth(1)
                }
            } else {
                if (cc == 1) {
                    cc = 0
                    canvas.setStrokeWidth(2)
                }
            }
            canvas.strokeLine(t[i].x * SCALE, t[i].y * SCALE, t[(i + Checkpoints / 50)].x * SCALE, t[(i + Checkpoints / 50)].y * SCALE)
        }

        canvas.setStrokeWidth(1)
        canvas.setStroke("FF0000")
        for (act in t.actions) {
            canvas.strokeCircle(t[act.checkNr].x * SCALE, t[act.checkNr].y * SCALE, AR)
        }
        canvas.setStroke("#FF00C3A0")
        drawVector(lk.vec(), Vec2d(LookaheadScale.x, LookaheadScale.y).rotated(-gangle(t.deriv(lastIndex).vec())), "#FF0000A0", 1, SCALE)
        drawVector(lk.vec(), Vec2d(-LookaheadScale.x, LookaheadScale.y).rotated(-gangle(t.deriv(lastIndex).vec())), "#FF0000A0", 1, SCALE)
        drawVector(lk.vec(), Vec2d(LookaheadScale.x, -LookaheadScale.y).rotated(-gangle(t.deriv(lastIndex).vec())), "#FF0000A0", 1, SCALE)
        drawVector(lk.vec(), Vec2d(-LookaheadScale.x, -LookaheadScale.y).rotated(-gangle(t.deriv(lastIndex).vec())), "#FF0000A0", 1, SCALE)

        drawVector(lk.vec(), Vec2d(LookaheadScale.x, LookaheadScale.y).rotated(gangle(t.deriv(lastIndex).vec())), "#FF0000A0", 1, SCALE)
        drawVector(lk.vec(), Vec2d(-LookaheadScale.x, LookaheadScale.y).rotated(gangle(t.deriv(lastIndex).vec())), "#FF0000A0", 1, SCALE)
        drawVector(lk.vec(), Vec2d(LookaheadScale.x, -LookaheadScale.y).rotated(gangle(t.deriv(lastIndex).vec())), "#FF0000A0", 1, SCALE)
        drawVector(lk.vec(), Vec2d(-LookaheadScale.x, -LookaheadScale.y).rotated(gangle(t.deriv(lastIndex).vec())), "#FF0000A0", 1, SCALE)
        canvas.setStroke("#FF20A350")
        canvas.strokeCircle(t.end.x * SCALE, t.end.y * SCALE, PPStartEnd * SCALE)
        canvas.setStroke("#F0B550A0")
        canvas.strokeCircle(t.end.x * SCALE, t.end.y * SCALE, t.peruStart * SCALE)
        canvas.setStroke("#F0B55050")
        canvas.strokeCircle(t.end.x * SCALE, t.end.y * SCALE, t.peruEnd * SCALE)
        canvas.setStroke("#3010FF30")
        canvas.strokeLine(p.x * SCALE, p.y * SCALE,
                (p.x * SCALE + speed * cos(angle + timmy.yaw) * SPC), (p.y * SCALE + speed * sin(angle + timmy.yaw) * SPC))
        canvas.setStrokeWidth(3)
        canvas.setStroke("#000000")
        canvas.fillCircle(lk.x * SCALE, lk.y * SCALE, lkr)
        canvas.setStrokeWidth(1)
        canvas.strokeLine(10.0, 10.0, 10.0 + tspeed * TSC * SCALE, 10.0)
    }

    var atLastt = false
    fun startFollowTraj(t: Trajectory) {
        ctraj = t
        ctraj.lastCompletedAction = 0
        lastIndex = 0
        done = false
        error = false
        haveTraj = true
        haveResetF = false
        atLast = false
        atLastt = false
        atLastT.reset()
        ep.reset()
        runningTime.reset()
        longP.reset()
        transP.reset()
        angleP.reset()
        log("ctraj", ctraj)
    }

    private fun gangle(o12: Vec2d): Double {
        val tan = if (epsEq(o12.x, 0.0)) 0.0 else atan(o12.y / o12.x)
        return if (o12.x > 0.0) {
            tan
        } else {
            tan + Math.PI
        }
    }

    private fun getPeruCoef() = clamp(((ctraj.end - cp).dist() - ctraj.peruStart) * (PeruMax - PeruMin) / (ctraj.peruEnd - ctraj.peruStart) + PeruMin, PeruMin, PeruMax)

    private var ep = ElapsedTime()
    private var cp = Pose()
    private var runningTime = ElapsedTime()
    private var atLast = false
    private var atLastT = ElapsedTime()
    private var haveResetF = false

    private val angleP = PID(PidAngle)
    private val transP = PID(PidTrans)
    private val longP = PID(PidLong)
    private val transFP = PID(PidFinalTrans)
    private val longFP = PID(PidFinalLong)

    private fun drawVector(pos: Vec2d, v: Vec2d, col: String, sz: Int, sc: Double) {
        val cv = RobotFuncs.tp.fieldOverlay()
        cv.setStrokeWidth(sz)
        cv.setStroke(col)
        cv.strokeLine(pos.x * SCALE, pos.y * SCALE, pos.x * SCALE + v.x * sc, pos.y * SCALE + v.y * sc)
    }

    private fun drawVector(pos: Vec2d, v: Vec2d, col: String) = drawVector(pos, v, col, 2, PPPPPPP)
    private fun drawVector(pos: Vec2d, v: Vec2d) = drawVector(pos, v, "#FF000070")

    fun update() { /// TODO: Add bump detection
        if (busy) {
            cp = localizer.pose
            val lk = lookahead(cp)

            if (lastIndex == Checkpoints && atLastt) {
                atLastt = true
                transFP.reset()
                longFP.reset()
            }

            if (!atLast) {
                if (if (ctraj.timeout < 0.001) (lastIndex == Checkpoints) else ((ctraj.end - cp).dist() < PPStartEnd)) {
                    atLast = true
                    atLastT.reset()
                    if (ctraj.timeout < 0.001) {
                        done = true
                        return
                    }
                }
            } else if (atLastT.seconds() > ctraj.timeout) {
                swerve.move(0.0, swerve.angle, 0.0)
                if (ctraj.timeout > 0.001) {
                    log("PurePursuitError", "Could not reach the required position in ${ctraj.timeout} (error = ${cp - lk}; Perp = ${(cp - lk).vec().rotated(-gangle(ctraj.deriv(lastIndex).vec()))})")
                }
                error = true
                return
            }

            if (abs(timmy.yawVel) < HAPPY_HEAD_VEL && abs(angDiff(ctraj.end.h, cp.h)) < HAPPY_HEAD && (ctraj.end - cp).dist() < HAPPY_DIST && localizer.poseVel.dist() < HAPPY_VEL) {
                swerve.move(0.0, swerve.angle, 0.0)
                done = true
                return
            }

            val pcoef = getPeruCoef()
            val trajAngle = gangle(ctraj.deriv(lastIndex).vec())
            val peru = (lk - cp).vec()
            val peruR = peru.rotated(-trajAngle) /// I LOVE MATH
            val peruTP: Double
            val peruLP: Double
            if (lastIndex == Checkpoints) {
                if (haveResetF) {
                    transFP.reset()
                    longFP.reset()
                }
                peruTP = abs(transFP.update(peruR.y)).coerceAtMost(2.0)
                peruLP = abs(longFP.update(peru.x)).coerceAtMost(2.0)
            } else {
                peruTP = abs(transP.update(peruR.y)).coerceAtMost(2.0)
                peruLP = abs(longP.update(peru.x)).coerceAtMost(2.0)
            }
            val peruCR = Vec2d(peruR.x * peruLP, peruR.y * peruTP)

            val peruC = peruCR.rotated(trajAngle)
            var angle = gangle(peruC) - timmy.yaw
            var peruV = Vec2d(peruTP, peruLP)
            val peruVd = peruV.dist()
            if (peruVd > 1.0) {
                peruV /= peruVd
            }
            val peruP = peruV.dist()
            /*
 __  __       _   _
|  \/  | __ _| |_| |__
| |\/| |/ _` | __| '_ \
| |  | | (_| | |_| | | |
|_|  |_|\__,_|\__|_| |_|

  _______
 / /___ /
/ /  |_ \
\ \ ___) |
 \_\____/
             */
            if (__LOG_STATUS) {
                log("TrajAngle", trajAngle)
                drawVector(ctraj[lastIndex].vec(), Vec2d(1.0, 0.0).rotated(trajAngle))
                drawVector(ctraj[lastIndex].vec(), Vec2d(0.0, 1.0).rotated(trajAngle))
                drawVector(cp.vec(), peru, "#0000FFB0")
                log("Peru", peru)
                drawVector(cp.vec(), peruR, "#00FF00B0")
                log("PeruR", peruR)
                log("PeruTP", peruTP)
                log("PeruLP", peruLP)
                log("PeruCR", peruCR)
                drawVector(cp.vec(), peruCR, "#FF0000B0")
                log("PeruC", peruC)
                drawVector(cp.vec(), peruC, "#000000B0")
                log("PeruP", peruP)
                log("Angle", angle)
                log("Angle-t", angle + timmy.yaw)
                log("0PeruCd2", peruC.dist2())
                log("0PeruCd", peruC.dist())
            }
            drawRobot()

            val tcoef = min(MAX_VEL, ctraj.initVel + MAX_ACC * runningTime.seconds()) * ctraj.maxFraction / MAX_VEL
            var speed = clamp(tcoef * peruP * pcoef + PPStaticSpeed, 0.0, PPMaxSpeed)
            var angPower = max(min(angleP.update(angDiff(cp.h, lk.h)), PPMaxAngPower), -PPMaxAngPower)
            if (speed.isNaN()) {
                log("PurePursuitError", "NaN speed")
                speed = 0.0
            }
            if (angle.isNaN()) {
                log("PurePursuitError", "NaN angle")
                angle = 0.0
            }
            if (angPower.isNaN()) {
                log("PurePursuitError", "NaN anglePower")
                angPower = 0.0
            }

            if (USE_AUTO_MOVE) {
                swerve.move(speed, angle, angPower)
            } else if (USE_SWERVE) {
                moveSwerve()
            }
            draw(ctraj, cp, lk, speed, angle, angPower)
            ep.reset()
            logs("S_lk", lk)
            logs("S_Dist", lk - cp)
            log("S_Pos", cp)
            log("S_FinalDist", ctraj.end - cp)
            logs("S_ctraje", ctraj.end)
            logs("lastIndex", lastIndex)
            logs("SWERVE_Tcoef", tcoef)
            logs("SWERVE_pspeed", angPower)
            logs("SWERVE_perCoef", pcoef)
        } else {
            if (USE_AUTO_MOVE) {
                swerve.move(0.0, swerve.angle, 0.0)
                swerve.lb.speed = 0.0
                swerve.rb.speed = 0.0
                swerve.lf.speed = 0.0
                swerve.rf.speed = 0.0
            } else if (USE_SWERVE) {
                moveSwerve()
            }
        }
    }

}
