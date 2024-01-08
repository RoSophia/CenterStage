package org.firstinspires.ftc.teamcode.hardware

import com.qualcomm.robotcore.util.ElapsedTime
import org.firstinspires.ftc.teamcode.utils.RobotFuncs.log
import org.firstinspires.ftc.teamcode.utils.RobotFuncs.logs
import org.firstinspires.ftc.teamcode.utils.Util.angDiff
import org.firstinspires.ftc.teamcode.utils.Util.angNorm
import kotlin.math.abs
import kotlin.math.sign

class PIDFC(@JvmField var p: Double, @JvmField var i: Double, @JvmField var d: Double, @JvmField var f: Double)

class CServo(val name: String, eoff: Double, gearr: Double, private val can360: Boolean, private val pd1: PIDFC, private val pd2: PIDFC, private val maxErr: Double) {
    constructor(name: String, eoff: Double, gearr: Double, can360: Boolean, pd1: PIDFC) : this(name, eoff, gearr, can360, pd1, pd1, 0.0)
    constructor(name: String, eoff: Double, gearr: Double, can360: Boolean) : this(name, eoff, gearr, can360, PIDFC(0.0, 0.0, 0.0, 0.0))
    constructor(name: String, eoff: Double, gearr: Double) : this(name, eoff, gearr, true)

    private val s = MCRServo(name + "S")
    val e = AbsEnc(name + "E", eoff, gearr)

    var err: Double = 0.0
    var der: Double = 0.0
    var cp: Double = 0.0
    val timer = ElapsedTime()

    var lastErr = 0.0
    var int = 0.0
    fun update() {
        cp = e.angle
        err = if (can360) {
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
        if (abs(err) > maxErr) {
            if (err >= 0.0) {
                s.power = sign(err) * pd1.f + err * pd1.p + der * pd1.d + int * pd1.i
            } else {
                s.power = sign(err) * pd2.f + err * pd2.p + der * pd2.d + int * pd2.i
            }
        } else {
            s.power = 0.0
        }

        logs("CServo_${name}_pidf1", "${pd1.f} - ${pd1.p} - ${pd1.d} - ${pd1.i}")
        logs("CServo_${name}_pidf2", "${pd2.f} - ${pd2.p} - ${pd2.d} - ${pd2.i}")
        logs("CServo_${name}_der", der)
        logs("CServo_${name}_Pow", s.power)
        logs("CServo_${name}_Timer", timer.seconds())
        logs("CServo_${name}_Enc", angNorm(cp))
        logs("CServo_${name}_Tar", angNorm(pt))
        timer.reset()
    }

    var pt: Double = 0.0

    fun init() {
        timer.reset()
    }

    fun close() {
    }
}
