package org.firstinspires.ftc.teamcode.hardware

import org.firstinspires.ftc.teamcode.utils.RobotFuncs.logs
import org.firstinspires.ftc.teamcode.utils.RobotVars._MOVE_SWERVE
import org.firstinspires.ftc.teamcode.utils.RobotVars.SERVO_GEAR_RATIO
import org.firstinspires.ftc.teamcode.utils.RobotVars.WheelPidLBB
import org.firstinspires.ftc.teamcode.utils.RobotVars.WheelPidLBF
import org.firstinspires.ftc.teamcode.utils.RobotVars.WheelPidLFB
import org.firstinspires.ftc.teamcode.utils.RobotVars.WheelPidLFF
import org.firstinspires.ftc.teamcode.utils.RobotVars.WheelPidRBB
import org.firstinspires.ftc.teamcode.utils.RobotVars.WheelPidRBF
import org.firstinspires.ftc.teamcode.utils.RobotVars.WheelPidRFB
import org.firstinspires.ftc.teamcode.utils.RobotVars.WheelPidRFF
import org.firstinspires.ftc.teamcode.utils.RobotVars.canInvertMotor
import org.firstinspires.ftc.teamcode.utils.Util.angDiff
import org.firstinspires.ftc.teamcode.utils.Util.angNorm
import org.firstinspires.ftc.teamcode.utils.Util.epsEq
import kotlin.math.PI
import kotlin.math.abs

class SwerveModule(val name: String, eoff: Double) {
    val s = CServo(name, eoff, SERVO_GEAR_RATIO, true,
            if (name[name.length - 1] == 'F')
                if (name[name.length - 2] == 'L')
                    WheelPidLFF
                else
                    WheelPidRFF
            else
                if (name[name.length - 2] == 'L')
                    WheelPidLBF
                else
                    WheelPidRBF,

            if (name[name.length - 1] == 'F')
                if (name[name.length - 2] == 'L')
                    WheelPidLFB
                else
                    WheelPidRFB
            else
                if (name[name.length - 2] == 'L')
                    WheelPidLBB
                else
                    WheelPidRBB
    )

    /*
       /__\
       |___|
       |___|
       |_I_|
       |___|
 _____ _____ _____
|__I__|_____|__R__|
       |___|
       |___|
       |___|
       |_N_|
       |___|
       |___|
       |___|
       |___|
  Doamne Fereste!
     */


    val m = Motor(name + "M", encoder = false, rev = false, overdrive = true)

    init {
        if (_MOVE_SWERVE) {
            s.init()
        }
        logs("ServoModule_${name}_Status", "Init")
    }

    fun update() {
        if (_MOVE_SWERVE) {
            s.update()
        }
    }

    var off = 0.0
    var angle: Double = 0.0
        set(v) {
            if (_MOVE_SWERVE) {
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
                    logs("ServoModule_${name}_MRever", m.reverse)
                }
            }
        }

    var speed: Double = 0.0
        set(v) {
            if (_MOVE_SWERVE) {
                if (!epsEq(v, field)) {
                    logs("ServoModule_${name}_MSpeed", v)
                    logs("ServoModule_${name}_MRever", m.reverse)
                    m.power = v
                    field = v
                }
            }
        }

    fun close() {
        if (_MOVE_SWERVE) {
            s.close()
        }
        logs("ServoModule_${name}_Status", "Close")
    }
}
