package org.firstinspires.ftc.teamcode.hardware

import org.firstinspires.ftc.teamcode.utils.PID
import org.firstinspires.ftc.teamcode.utils.RobotFuncs.logs
import org.firstinspires.ftc.teamcode.utils.RobotVars.___C
import org.firstinspires.ftc.teamcode.utils.Util.angDiff
import kotlin.math.abs

class PIDFC(@JvmField var p: Double, @JvmField var i: Double, @JvmField var d: Double, @JvmField var f: Double) {
    override fun toString() = "($p $i $d $f)"
}

class CServo(val name: String, eoff: Double, private val pd: PIDFC, val id: Int) {
    private val s = MCRServo(name + "S")
    val e = AbsEnc(name + "E", eoff)

    private var cp: Double = 0.0
    var pt: Double = 0.0

    private val pidf = PID(pd)
    fun update() {
        cp = e.angle
        val err = angDiff(pt, cp)
        val pwr = pidf.update(err)
        logs("${name}_cp", cp)
        logs("${name}_er", err)
        logs("${name}_PID", pd)
        logs("${name}_Pwr", pwr)

        ___C[id] = abs(pwr)
        s.power = pwr
    }
}
