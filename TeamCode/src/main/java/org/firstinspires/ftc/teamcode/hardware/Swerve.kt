package org.firstinspires.ftc.teamcode.hardware

import com.qualcomm.robotcore.util.ElapsedTime
import org.firstinspires.ftc.teamcode.utils.Pose
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
import kotlin.math.min
import kotlin.math.sin


class Swerve {
    val lf = SwerveModule("LF", OFFLF)
    val lb = SwerveModule("LB", OFFLB)
    val rf = SwerveModule("RF", OFFRF)
    val rb = SwerveModule("RB", OFFRB)
    private val modules = arrayListOf(lf, rf, rb, lb)
    // LF RF RB LB

    private var ws = DoubleArray(4)
    private var wa = DoubleArray(4)
    private var maxs = 0.0
    var locked = false

    private fun getkms(kms: Double) = clamp((kms - WheelAlignStart) * (WheelAlignMax - WheelAlignMin) / (WheelAlignEnd - WheelAlignStart) + WheelAlignMin, WheelAlignMin, WheelAlignMax)

    val ep = ElapsedTime()
    private fun GETKMS(i: Int) = modules[i].latentImpulse * WheelsLBias[i] + abs(modules[i].speed) * (if (modules[i].speed >= 0) WheelsFBias[i] else WheelsBBias[i])

    fun update() {
        for (i in 0..3) {
            val cs = if (MOVE_SWERVE_MOTORS) {
                if (maxs > 1.0) {
                    ws[i] / maxs
                } else {
                    ws[i]
                } * getkms(PI / 2 - abs(angDiff(modules[i].angle, angNorm(modules[i].s.e.angle + modules[i].off))))
            } else {
                0.0
            }
            modules[i].tem = WheelsLatentVars[i]
            modules[i].speed = cs
            val li = modules[i].updateLatent()
            modules[i].latentImpulse = cs - li
            modules[i].angle = wa[i]
            modules[i].forcedForce = GETKMS(i)
            logs("AngDiff_$i", angDiff(modules[i].angle, angNorm(modules[i].s.e.angle + modules[i].off)))
            logs("LatentImpulse_$i", modules[i].latentImpulse)
            modules[i].update()
        }
        ep.reset()
    }

    fun start() {}

    fun stop() {}

    var speed = 0.0
    var angle = 0.0
    var turnPower = 0.0

    fun kmskms(pose: Pose) {
        val x: Double = pose.x
        val y: Double = pose.y
        val head: Double = pose.h

        val r = hypot(TRACK_WIDTH, WHEEL_BASE)
        val a: Double = x - head * (WHEEL_BASE / r) * HEADP
        val b: Double = x + head * (WHEEL_BASE / r) * HEADP
        val c: Double = y - head * (TRACK_WIDTH / r) * HEADP
        val d: Double = y + head * (TRACK_WIDTH / r) * HEADP

        if (locked) {
            ws = doubleArrayOf(0.0, 0.0, 0.0, 0.0)
            val PI2 = -Math.PI / 4
            wa = doubleArrayOf(PI2, 3 * PI2, 5 * PI2, 7 * PI2)
        } else {
            ws = doubleArrayOf(hypot(b, c), hypot(b, d), hypot(a, d), hypot(a, c))
            wa = doubleArrayOf(
                    atan2(b, c) + if (head > 0.0) WheelULF else WheelDLF,
                    atan2(b, d) + if (head > 0.0) WheelURF else WheelDRF,
                    atan2(a, d) + if (head > 0.0) WheelURB else WheelDRB,
                    atan2(a, c) + if (head > 0.0) WheelULB else WheelDLB)
        }

        maxs = max(ws[0], max(ws[1], max(ws[2], ws[3])))
    }

    fun move(speed: Double, angle: Double, turnPower: Double) {
        if (epsEq(this.speed, speed) && epsEq(this.angle, angle) && epsEq(this.turnPower, turnPower)) {
            return
        }
        log("Swerve_Movement", String.format("%.3f@%.3f : %.3f", speed, angle, turnPower))
        val actAng = angle + turnPower * SwerveAngP
        if (abs(speed) < 0.1 && abs(turnPower) < 0.1) {
            ws = doubleArrayOf(0.0, 0.0, 0.0, 0.0)
        } else {
            kmskms(Pose(sin(actAng) * speed, cos(actAng) * speed, turnPower * PI / KMSCONF))
        }
    }

    fun close() {
        lf.close()
        lb.close()
        rf.close()
        rb.close()
    }

}
