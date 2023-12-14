package org.firstinspires.ftc.teamcode.hardware

import com.qualcomm.robotcore.hardware.AnalogInput
import org.firstinspires.ftc.teamcode.utils.RobotFuncs
import org.firstinspires.ftc.teamcode.utils.RobotFuncs.log
import org.firstinspires.ftc.teamcode.utils.RobotVars.SERVO_GEAR_RATIO
import org.firstinspires.ftc.teamcode.utils.Util.angNorm
import kotlin.math.PI

class AbsEnc(private val name: String, private val off: Double) {
    private val enc: AnalogInput = RobotFuncs.hardwareMap.get(AnalogInput::class.java, name)

    private val maxVoltage = 3.305
    private val angPer01 = (PI * 2 * SERVO_GEAR_RATIO)

    val pos: Double
        get() {
            val cv = enc.voltage / maxVoltage
            log("AbsEnc_${name}_Volt", cv)
            return angNorm(cv * angPer01 + off)
        }
}