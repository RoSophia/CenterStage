package org.firstinspires.ftc.teamcode.hardware

import com.qualcomm.robotcore.util.ElapsedTime
import org.firstinspires.ftc.teamcode.utils.RobotVars.SwerveMaxPower
import org.firstinspires.ftc.teamcode.utils.RobotVars.__SwerveMove
import org.firstinspires.ftc.teamcode.utils.RobotVars.SwerveCanInvertMotor
import org.firstinspires.ftc.teamcode.utils.RobotVars.SwervePids
import org.firstinspires.ftc.teamcode.utils.RobotVars.SwerveWheelOffsets
import org.firstinspires.ftc.teamcode.utils.Util.angDiff
import org.firstinspires.ftc.teamcode.utils.Util.angNorm
import org.firstinspires.ftc.teamcode.utils.Util.epsEq
import kotlin.math.PI
import kotlin.math.abs

class SwerveModule(val name: String, id: Int) {
    val s = CServo(name, SwerveWheelOffsets[id], SwervePids[id], id)

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

    fun update() {
        if (__SwerveMove) {
            s.update()
        }
    }

    var off = 0.0
    var angle: Double = 0.0
        set(v) {
            if (__SwerveMove) {
                val vn = angNorm(v)
                val dif = angDiff(vn, field)
                if (!epsEq(dif, 0.0)) {
                    val actualDif = angDiff(vn + off, s.e.angle)
                    if (SwerveCanInvertMotor && abs(actualDif) >= (PI / 2)) {
                        m.reverse = !m.reverse
                        off = PI - off
                    }
                    s.pt = angNorm(vn + off)
                    field = v
                }
            }
        }

    val ep = ElapsedTime()

    var speed: Double = 0.0
        set(v) {
            if (__SwerveMove) {
                if (!epsEq(v, field)) {
                    m.power = v * SwerveMaxPower
                    ep.reset()
                    field = v
                }
            }
        }

}
