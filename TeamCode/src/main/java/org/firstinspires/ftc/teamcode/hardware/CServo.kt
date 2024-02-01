package org.firstinspires.ftc.teamcode.hardware

import org.firstinspires.ftc.teamcode.utils.PID
import org.firstinspires.ftc.teamcode.utils.RobotFuncs.log
import org.firstinspires.ftc.teamcode.utils.Util.angDiff

class PIDFC(@JvmField var p: Double, @JvmField var i: Double, @JvmField var d: Double, @JvmField var f: Double)

class CServo(val name: String, eoff: Double, pd: PIDFC) {
    private val s = MCRServo(name + "S")
    val e = AbsEnc(name + "E", eoff)

    private var cp: Double = 0.0
    var pt: Double = 0.0

    val pidf = PID(pd)
    fun updatef(forc: Double) {
        cp = e.angle
        val err = angDiff(pt, cp)
        val pwr = pidf.update(err)

        s.power = pwr + forc
    }

    fun update() = updatef(0.0)
}
