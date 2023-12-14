package org.firstinspires.ftc.teamcode.hardware

import org.firstinspires.ftc.teamcode.utils.RobotFuncs.log
import org.firstinspires.ftc.teamcode.utils.RobotFuncs.logs
import org.firstinspires.ftc.teamcode.utils.RobotVars.MOVE_SWERVE
import org.firstinspires.ftc.teamcode.utils.RobotVars.canInvertMotor
import org.firstinspires.ftc.teamcode.utils.Util.angDiff
import org.firstinspires.ftc.teamcode.utils.Util.angNorm
import org.firstinspires.ftc.teamcode.utils.Util.epsEq
import kotlin.math.PI
import kotlin.math.abs

class SwerveModule(val name: String, eoff: Double) {
    val s = CServo(name, eoff)
    private val m = Motor(name + "M", encoder = false, rev = false, overdrive = true)

    init {
        if (MOVE_SWERVE) {
            s.initPid()
        }
        logs("ServoModule_${name}_Status", "Init")
    }

    var off = 0.0
    var angle: Double = 0.0
        set(v) {
            if (MOVE_SWERVE) {
                val vn = angNorm(v)
                val dif = angDiff(vn, field)
                if (!epsEq(dif, 0.0)) {
                    val actualDif = angDiff(vn + off, s.e.pos)
                    if (canInvertMotor && abs(actualDif) >= (PI / 2)) {
                        m.reverse = !m.reverse
                        off = PI - off
                    }
                    s.pt = angNorm(vn + off)
                    field = v
                    log("ServoModule_${name}_MRever", m.reverse)
                }
            }
        }

    var speed: Double = 0.0
        set(v) {
            if (MOVE_SWERVE) {
                if (!epsEq(v, field)) {
                    log("ServoModule_${name}_MSpeed", v)
                    log("ServoModule_${name}_MRever", m.reverse)
                    m.power = v
                    field = v
                }
            }
        }

    fun close() {
        if (MOVE_SWERVE) {
            s.joinPid()
        }
        logs("ServoModule_${name}_Status", "Close")
    }
}
