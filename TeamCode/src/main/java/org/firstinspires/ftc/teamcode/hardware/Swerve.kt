package org.firstinspires.ftc.teamcode.hardware

import com.qualcomm.robotcore.util.ElapsedTime
import org.firstinspires.ftc.teamcode.utils.Pose
import org.firstinspires.ftc.teamcode.utils.RobotFuncs.KILLALL
import org.firstinspires.ftc.teamcode.utils.RobotFuncs.log
import org.firstinspires.ftc.teamcode.utils.RobotFuncs.logs
import org.firstinspires.ftc.teamcode.utils.RobotVars.HEADP
import org.firstinspires.ftc.teamcode.utils.RobotVars.KMSCONF
import org.firstinspires.ftc.teamcode.utils.RobotVars.OFFLB
import org.firstinspires.ftc.teamcode.utils.RobotVars.OFFLF
import org.firstinspires.ftc.teamcode.utils.RobotVars.OFFRB
import org.firstinspires.ftc.teamcode.utils.RobotVars.OFFRF
import org.firstinspires.ftc.teamcode.utils.RobotVars.SwerveAngP
import org.firstinspires.ftc.teamcode.utils.RobotVars.TRACK_WIDTH
import org.firstinspires.ftc.teamcode.utils.RobotVars.WHEEL_BASE
import org.firstinspires.ftc.teamcode.utils.RobotVars.WheelDLB
import org.firstinspires.ftc.teamcode.utils.RobotVars.WheelDLF
import org.firstinspires.ftc.teamcode.utils.RobotVars.WheelDRB
import org.firstinspires.ftc.teamcode.utils.RobotVars.WheelDRF
import org.firstinspires.ftc.teamcode.utils.RobotVars.WheelULB
import org.firstinspires.ftc.teamcode.utils.RobotVars.WheelULF
import org.firstinspires.ftc.teamcode.utils.RobotVars.WheelURB
import org.firstinspires.ftc.teamcode.utils.RobotVars.WheelURF
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
    val lf = SwerveModule("LF", OFFLF)
    val lb = SwerveModule("LB", OFFLB)
    val rf = SwerveModule("RF", OFFRF)
    val rb = SwerveModule("RB", OFFRB)
    val modules = arrayListOf(lf, rf, rb, lb)
    var ws = DoubleArray(4)
    var wa = DoubleArray(4)
    var maxs = 0.0
    var maintainHeading = false
    var locked = false

    var motorThread: Thread
    var trunning = false
    var csp: Double = 0.0

    fun getkms(kms: Double): Double {
        return clamp(1.4 - kms, 0.0, 1.0)
    }

    init {
        logs("Swerve_Status", "Init");
        trunning = true
        motorThread = Thread {
            val ep = ElapsedTime()
            ep.reset()
            while (trunning && !KILLALL) {
                for (i in 0..3) {
                    modules[i].angle = wa[i]
                    log("AngDiff_$i", angDiff(modules[i].s.e.pos + modules[i].off, modules[i].angle))
                    log("PAVER_$i", modules[i].m.current)
                    modules[i].speed =
                            if (maxs > 1.0) {
                                ws[i] / maxs
                            } else {
                                ws[i]
                            }

                    /*
                            if (maxs > 1.0) {
                                getkms(abs(angDiff(modules[i].s.e.pos + modules[i].off, modules[i].angle))) * ws[i] / maxs
                            } else {
                                getkms(abs(angDiff(modules[i].s.e.pos + modules[i].off, modules[i].angle))) * ws[i]
                            }
                     */
                    // modules[i].speed = if (abs(angDiff(modules[i].s.e.pos + modules[i].off, modules[i].angle)) <= 0.3) { if (maxs > 1.0) { ws[i] / maxs } else { ws[i] } } else { 0.0 }
                }
            }
        }
    }

    fun start() {
        trunning = true
        motorThread.start()
    }

    fun stop() {
        trunning = false
        motorThread.join()
    }

    fun turn(turnPower: Double) {
        lf.angle = angNorm(PI * 3 / 4)
        lb.angle = angNorm(PI * 5 / 4)
        rf.angle = angNorm(PI * 1 / 4)
        rb.angle = angNorm(PI * 7 / 4)
        csp = turnPower
    }

    var speed = 0.0
    var angle = 0.0
    var turnPower = 0.0


    fun kmskms(pose: Pose) {
        val x: Double = pose.x
        val y: Double = pose.y
        val head: Double = pose.h

        // LF RF RB LB
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
            if (!maintainHeading) wa = doubleArrayOf(
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
        log("Swerve_Movement", "${speed}@${angle} tp: $turnPower")
        val actAng = angle + turnPower * SwerveAngP
        if (abs(speed) < 0.1 && abs(turnPower) < 0.1) {
            ws = doubleArrayOf(0.0, 0.0, 0.0, 0.0)
        } else {
            kmskms(Pose(sin(actAng) * speed, cos(actAng) * speed, turnPower * PI / KMSCONF))
        }
    }

    fun close() {
        logs("Swerve_Status", "InitClose");
        lf.close()
        lb.close()
        rf.close()
        rb.close()

        logs("Swerve_Status", "FinishClose");
    }

}
