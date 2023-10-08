package org.firstinspires.ftc.teamcode.hardware

import org.firstinspires.ftc.teamcode.utils.RobotFuncs.log
import org.firstinspires.ftc.teamcode.utils.RobotVars.canInvertMotor
import org.firstinspires.ftc.teamcode.utils.Util.angDiff
import org.firstinspires.ftc.teamcode.utils.Util.epsEq
import org.firstinspires.ftc.teamcode.utils.Util.mod
import kotlin.math.PI
import kotlin.math.abs

class SwerveModule(val name: String, eoff: Double) {
    private val s = CServo(name, eoff)
    private val m = Motor(name + "M", encoder = false, rev = false, overdrive = true)

    init {
        s.initPid()
        log("ServoModule_${name}_Status", "Init")
    }

    var angle: Double = 0.0
        set(v) {
            if (!epsEq(v, field)) {
                val ov = mod(v + PI, PI * 2)
                if (!canInvertMotor || abs(angDiff(v, field)) < abs(angDiff(ov, field))) {
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
                log("ServoModule_${name}_MSpeed", "$v")
                log("ServoModule_${name}_MRever", "${m.reverse}")
                m.power = v
                field = v
            }
        }

    fun close() {
        s.joinPid()
        log("ServoModule_${name}_Status", "Close")
    }
}
