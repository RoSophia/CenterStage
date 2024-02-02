package org.firstinspires.ftc.teamcode.utils

import com.qualcomm.robotcore.util.ElapsedTime
import org.firstinspires.ftc.teamcode.hardware.PIDFC
import kotlin.math.sign

// Pid ca-mi bag ceva in a mai scrie tot codu de pid inca odata

class PID(private val coef: PIDFC) {
    private var lp = 0.0
    var ep = ElapsedTime()
    var vi = 0.0

    init { ep.reset() }

    fun reset() {
        ep.reset()
        vi = 0.0
    }

    fun update(err: Double): Double {
        val d = (err - lp) / ep.seconds()
        vi += err * ep.seconds()
        ep.reset()
        lp = err
        return err * coef.p + d * coef.d + vi * coef.i + sign(err) * coef.f
    }
}