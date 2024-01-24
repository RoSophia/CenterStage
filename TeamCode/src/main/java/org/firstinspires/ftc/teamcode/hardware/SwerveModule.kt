package org.firstinspires.ftc.teamcode.hardware

import com.qualcomm.robotcore.util.ElapsedTime
import org.firstinspires.ftc.teamcode.utils.RobotVars.SwerveMaxPower
import org.firstinspires.ftc.teamcode.utils.RobotVars.WheelsMaxErr
import org.firstinspires.ftc.teamcode.utils.RobotVars.WheelsPidB
import org.firstinspires.ftc.teamcode.utils.RobotVars.WheelsPidF
import org.firstinspires.ftc.teamcode.utils.RobotVars._MOVE_SWERVE
import org.firstinspires.ftc.teamcode.utils.RobotVars.canInvertMotor
import org.firstinspires.ftc.teamcode.utils.Util.angDiff
import org.firstinspires.ftc.teamcode.utils.Util.angNorm
import org.firstinspires.ftc.teamcode.utils.Util.epsEq
import kotlin.math.PI
import kotlin.math.abs
import kotlin.math.min

class SwerveModule(val name: String, eoff: Double) {
    val s = CServo(name, eoff, true,
            if (name[name.length - 1] == 'F')
                WheelsPidF
            else
                WheelsPidB,
            WheelsMaxErr
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


    val m = Motor(name + "M", encoder = false, rev = name == "LF", overdrive = true)

    init {
        if (_MOVE_SWERVE) {
            s.init()
        }
    }

    var forcedForce = 0.0
    fun update() {
        if (_MOVE_SWERVE) {
            s.updatef(forcedForce)
        }
    }

    var off = 0.0
    var angle: Double = 0.0
        set(v) {
            if (_MOVE_SWERVE) {
                val vn = angNorm(v)
                val dif = angDiff(vn, field)
                if (!epsEq(dif, 0.0)) {
                    val actualDif = angDiff(vn + off, s.e.angle)
                    if (canInvertMotor && abs(actualDif) >= (PI / 2)) {
                        m.reverse = !m.reverse
                        off = PI - off
                    }
                    s.pt = angNorm(vn + off)
                    field = v
                }
            }
        }

    var lspeed = 0.0
    var tem = 0.0
    val ep = ElapsedTime()
    fun updateLatent() = lspeed + (speed - lspeed) * min(abs(ep.seconds() * tem / (speed - lspeed)), 1.0)

    var latentImpulse: Double = 0.0
    var speed: Double = 0.0
        set(v) {
            if (_MOVE_SWERVE) {
                if (!epsEq(v, field)) {
                    m.power = v * SwerveMaxPower
                    lspeed = updateLatent()
                    ep.reset()
                    field = v
                }
            }
        }

    fun close() {
        if (_MOVE_SWERVE) {
            s.close()
        }
    }
}
