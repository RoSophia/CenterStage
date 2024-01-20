package org.firstinspires.ftc.teamcode.hardware

import com.qualcomm.robotcore.util.ElapsedTime
import org.firstinspires.ftc.teamcode.utils.RobotFuncs.log
import org.firstinspires.ftc.teamcode.utils.RobotFuncs.logs
import org.firstinspires.ftc.teamcode.utils.Util.angDiff
import org.firstinspires.ftc.teamcode.utils.Util.angNorm
import org.firstinspires.ftc.teamcode.utils.Util.clamp
import kotlin.math.abs
import kotlin.math.sign

class PIDFC(@JvmField var p: Double, @JvmField var i: Double, @JvmField var d: Double, @JvmField var f: Double, @JvmField var k: Double) {
    constructor(p: Double, i: Double, d: Double, f: Double) : this(p, i, d, f, 0.0)
}

class CServo(val name: String, eoff: Double, private val can360: Boolean, private val pd: PIDFC, val pcoef: Double) {
    constructor(name: String, eoff: Double, can360: Boolean, pd1: PIDFC) : this(name, eoff, can360, pd1, 1.0)
    constructor(name: String, eoff: Double, can360: Boolean) : this(name, eoff, can360, PIDFC(0.0, 0.0, 0.0, 0.0))
    constructor(name: String, eoff: Double, ) : this(name, eoff, true)

    private val s = MCRServo(name + "S")
    val e = AbsEnc(name + "E", eoff)

    var err: Double = 0.0
    var der: Double = 0.0
    var cp: Double = 0.0
    val timer = ElapsedTime()

    var pt: Double = 0.0

    var lastErr = 0.0
    var int = 0.0
    fun updatef(forc: Double) {
        cp = e.angle
        err = if (can360) {
            pt = angNorm(pt)
            cp = angNorm(cp)
            angDiff(pt, cp)
        } else {
            pt - cp
        }
        if (abs(err) < 0.02) {
            err = 0.0
            lastErr = 0.0
        }
        der = if (can360) {
            angDiff(err, lastErr) / timer.seconds()
        } else {
            (err - lastErr) / timer.seconds()
        }
        int += (err * timer.seconds())
        lastErr = err

        if (can360 && err > 3.0) {
            err = 0.0
            der = 0.0
            int = 0.0
        }

        val f = err * pd.p + int * pd.i + der * pd.d + sign(err) * pd.f
        val tp = clamp(f, -1.0, 1.0) /// KOky what the zuc

        s.power = (tp + if (abs(err) > 0.02) (pd.k * sign(tp)) else 0.0) + forc
        //s.power = (tp + if (abs(err) > 0.02) (pd.k * sign(tp)) else 0.0)

        if (name == "LF") {
            log("CServo_${name}_pd", "${pd.f} - ${pd.p} - ${pd.d} - ${pd.i}")
            log("CServo_${name}_forc", forc)
            log("CServo_${name}_err", err)
            log("CServo_${name}_der", der)
            log("CServo_${name}_Pow", s.power)
            log("CServo_${name}_Timer", timer.seconds())
            log("CServo_${name}_Enc", cp)
            log("CServo_${name}_Tar", pt)
        } else {
            logs("CServo_${name}_pd", "${pd.f} - ${pd.p} - ${pd.d} - ${pd.i}")
            logs("CServo_${name}_err", err)
            logs("CServo_${name}_der", der)
            logs("CServo_${name}_Pow", s.power)
            logs("CServo_${name}_Timer", timer.seconds())
            logs("CServo_${name}_Enc", cp)
            logs("CServo_${name}_Tar", pt)
        }
        timer.reset()
    }

    fun update() = updatef(0.0)


    fun init() {
        timer.reset()
    }

    fun close() {
    }
}
