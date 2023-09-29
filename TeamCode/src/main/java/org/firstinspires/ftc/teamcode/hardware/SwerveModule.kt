package org.firstinspires.ftc.teamcode.hardware

import org.firstinspires.ftc.teamcode.utils.Util.angDiff
import org.firstinspires.ftc.teamcode.utils.Util.epsEq
import org.firstinspires.ftc.teamcode.utils.Util.mod
import kotlin.math.PI
import kotlin.math.abs

class SwerveModule(name: String) {
    private val s = CServo(name + "S", name + "E")
    private val m = Motor(name + "M", encoder = false, rev = false, overdrive = true)

    init {
        s.initPid()
    }

    var angle: Double = 0.0
        set(v) {
            if (!epsEq(v, field)) {
                val ov = mod(v + PI, PI * 2)
                if (abs(angDiff(v, field)) < abs(angDiff(ov, field))) {
                    s.pt = v
                    field = v
                } else {
                    s.pt = ov
                    field = ov
                    m.reverse = !m.reverse
                }
            }
        }

    var speed: Double = 0.0
        set(v) {
            if (!epsEq(v, field)) {
                m.power = v
                field = v
            }
        }

    fun close() {
        s.joinPid()
    }
}