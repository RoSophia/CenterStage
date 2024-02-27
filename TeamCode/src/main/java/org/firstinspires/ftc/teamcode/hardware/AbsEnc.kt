package org.firstinspires.ftc.teamcode.hardware

import com.qualcomm.robotcore.hardware.AnalogInput
import org.firstinspires.ftc.teamcode.utils.RobotFuncs
import org.firstinspires.ftc.teamcode.utils.RobotFuncs.log
import org.firstinspires.ftc.teamcode.utils.RobotFuncs.logs
import org.firstinspires.ftc.teamcode.utils.RobotVars.EncoderAccelFuckery
import org.firstinspires.ftc.teamcode.utils.RobotVars.EncoderPowerFuckery
import org.firstinspires.ftc.teamcode.utils.RobotVars.___CURRENT_SCHWERVE_ACCEL
import org.firstinspires.ftc.teamcode.utils.RobotVars.___CURRENT_SCHWERVE_SWPEED
import org.firstinspires.ftc.teamcode.utils.Util.angNorm
import kotlin.math.PI
import kotlin.math.max
import kotlin.math.min

class AbsEnc(private val name: String, private val off: Double, private val inverted: Boolean)  {
    constructor(name: String, off: Double) : this (name, off, false)
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
            val ccv = enc.voltage
            maxVoltage = min(max(maxVoltage, ccv), 3.3)
            val v = if (inverted) -ccv else ccv + ___CURRENT_SCHWERVE_SWPEED * EncoderPowerFuckery + ___CURRENT_SCHWERVE_ACCEL * EncoderAccelFuckery
            logs("${name}_cv", v)

            val cv = v / maxVoltage
            return angNorm(cv * angPer01 + off)
        }
}