package org.firstinspires.ftc.teamcode.hardware

import com.qualcomm.robotcore.hardware.AnalogInput
import org.firstinspires.ftc.teamcode.utils.RobotFuncs
import org.firstinspires.ftc.teamcode.utils.RobotFuncs.logs
import org.firstinspires.ftc.teamcode.utils.Util.angNorm
import kotlin.math.PI
import kotlin.math.max
import kotlin.math.min

class AbsEnc(private val name: String, private val off: Double) {
    val enc: AnalogInput = RobotFuncs.hardwareMap.get(AnalogInput::class.java, name)

    private var maxVoltage = when (name) {
        "LFE" -> 3.266
        "LBE" -> 3.277
        "RFE" -> 3.286
        else  -> 3.279
    }
    private val angPer01 = PI * 2

    val angle: Double
        get() {
            val v = enc.voltage
            maxVoltage = min(max(maxVoltage, v), 3.3)
            val cv = v / maxVoltage
            logs("AbsEnc_${name}_AcVolt", v)
            logs("AbsEnc_${name}_MaxVolt", maxVoltage)
            logs("AbsEnc_${name}_Volt", cv)
            logs("AbsEnc_${name}_out", angNorm(cv * angPer01 + off))
            return angNorm(cv * angPer01 + off)
        }
}