package org.firstinspires.ftc.teamcode.hardware

import com.qualcomm.robotcore.hardware.AnalogInput
import org.firstinspires.ftc.teamcode.utils.RobotFuncs
import org.firstinspires.ftc.teamcode.utils.RobotFuncs.log
import org.firstinspires.ftc.teamcode.utils.RobotVars.SERVO_GEAR_RATIO
import org.firstinspires.ftc.teamcode.utils.Util.angNorm
import kotlin.math.PI

class AbsEnc(private val name: String, private val off: Double) {
    private val enc: AnalogInput = RobotFuncs.hardwareMap.get(AnalogInput::class.java, name)

    private var nturns = 0
    private val maxVoltage = enc.maxVoltage + 0.005
    private var lv = enc.voltage
    private val angPer01 = (PI * 2 * SERVO_GEAR_RATIO)

    val pos: Double
        get() {
            val cv = enc.voltage
            if (cv > 2.7 && lv < 1.0) {
                --nturns
            } else if (cv < 1.0 && lv > 2.7) {
                ++nturns
            }
            lv = cv

            log("AbsEnc_${name}_Volt", "$cv")
            log("AbsEnc_${name}_NT", "$nturns")

            val c01 = cv / maxVoltage
            log("AbsEnc_${name}_01M", "${(nturns + c01) * angPer01}")
            return angNorm((nturns + c01) * angPer01 + off)
        }
}