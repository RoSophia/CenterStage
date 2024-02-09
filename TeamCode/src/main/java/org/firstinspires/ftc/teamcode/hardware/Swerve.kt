package org.firstinspires.ftc.teamcode.hardware

import com.qualcomm.robotcore.util.ElapsedTime
import org.firstinspires.ftc.teamcode.utils.Pose
import org.firstinspires.ftc.teamcode.utils.RobotFuncs
import org.firstinspires.ftc.teamcode.utils.RobotFuncs.log
import org.firstinspires.ftc.teamcode.utils.RobotFuncs.logs
import org.firstinspires.ftc.teamcode.utils.RobotVars.*
import org.firstinspires.ftc.teamcode.utils.Util.angDiff
import org.firstinspires.ftc.teamcode.utils.Util.angNorm
import org.firstinspires.ftc.teamcode.utils.Util.clamp
import org.firstinspires.ftc.teamcode.utils.Util.epsEq
import kotlin.math.PI
import kotlin.math.abs
import kotlin.math.atan2
import kotlin.math.cos
import kotlin.math.hypot
import kotlin.math.max
import kotlin.math.sin


class Swerve {
    val lf = SwerveModule("LF", 0)
    val rf = SwerveModule("RF", 1)
    val rb = SwerveModule("RB", 2)
    val lb = SwerveModule("LB", 3)
    private val modules = arrayListOf(lf, rf, rb, lb)
    // LF RF RB LB

    private var ws = DoubleArray(4)
    private var wa = DoubleArray(4)
    private var maxs = 0.0
    var locked = false

    private fun getkms(kms: Double) = clamp((kms - WheelsAlignStart) * (WheelsAlignMax - WheelsAlignMin) / (WheelsAlignEnd - WheelsAlignStart) + WheelsAlignMin, WheelsAlignMin, WheelsAlignMax)

    val ep = ElapsedTime()
    fun update() {
        for (i in 0..3) {
            val cs = if (USE_SWERVE_MOTORS) {
                if (maxs > 1.0) {
                    ws[i] / maxs
                } else {
                    ws[i]
                } * getkms(PI / 2 - abs(angDiff(modules[i].angle, angNorm(modules[i].s.e.angle + modules[i].off))))
            } else {
                0.0
            }
            modules[i].angle = wa[i]
            modules[i].speed = cs + if (cs > 0.001) SwerveStaticRotation[i] else 0.0
            modules[i].update()
        }
        ep.reset()
    }

    var speed = 0.0
    var angle = 0.0
    var turnPower = 0.0

    private fun kmskms(pose: Pose) {
        val x: Double = pose.x
        val y: Double = pose.y
        val head: Double = pose.h

        val r = hypot(SwerveTrackWidth, SwerveWheelBase)
        val a: Double = x - head * (SwerveWheelBase / r) * SwerveHeadP
        val b: Double = x + head * (SwerveWheelBase / r) * SwerveHeadP
        val c: Double = y - head * (SwerveTrackWidth / r) * SwerveHeadP
        val d: Double = y + head * (SwerveTrackWidth / r) * SwerveHeadP

        if (locked) {
            ws = doubleArrayOf(0.0, 0.0, 0.0, 0.0)
            val PI2 = -Math.PI / 4
            wa = doubleArrayOf(PI2, 3 * PI2, 5 * PI2, 7 * PI2)
        } else {
            ws = doubleArrayOf(hypot(b, c), hypot(b, d), hypot(a, d), hypot(a, c))
            wa = doubleArrayOf(
                    atan2(b, c),
                    atan2(b, d),
                    atan2(a, d),
                    atan2(a, c))
        }

        maxs = max(ws[0], max(ws[1], max(ws[2], ws[3])))
    }

    private val AntiRetardationTimer = ElapsedTime()
    private var AntiRetardationFrames = 0
    private var accelTimer = ElapsedTime()
    fun move(speed: Double, angle: Double, turnPower: Double) {
        if (epsEq(this.speed, speed) && epsEq(this.angle, angle) && epsEq(this.turnPower, turnPower)) {
            log("KILIKIL", String.format("%.5f - %.5f : %.5f", speed, angle, turnPower))
            //return
        }
        log("Swerve_Movement", String.format("%.3f@%.3f : %.3f", speed, angle, turnPower))
        val actAng = angle + turnPower * SwerveAngP
        if (abs(speed) < 0.0002 && abs(turnPower) < 0.0002) {
            ws = if (AntiRetardationTimer.seconds() < SwerveAntiRetardationTime || AntiRetardationFrames < 2) {
                doubleArrayOf(SwerveAntiRetardationForce, SwerveAntiRetardationForce, SwerveAntiRetardationForce, SwerveAntiRetardationForce)
            } else {
                doubleArrayOf(0.0, 0.0, 0.0, 0.0)
            }
            ++AntiRetardationFrames
        } else {
            AntiRetardationTimer.reset()
            AntiRetardationFrames = 0
            kmskms(Pose(sin(actAng) * speed, cos(actAng) * speed, turnPower * PI / SwerveKmsConf))
        }

        val css = (clamp(abs(ws[0]), 0.0, 1.0) +
                   clamp(abs(ws[1]), 0.0, 1.0) +
                   clamp(abs(ws[2]), 0.0, 1.0) +
                   clamp(abs(ws[3]), 0.0, 1.0)) / 4
        ___CURRENT_SCHWERVE_ACCEL = abs(___CURRENT_SCHWERVE_SWPEED - css) / accelTimer.seconds()
        accelTimer.reset()
        ___CURRENT_SCHWERVE_SWPEED = css

    }
}
