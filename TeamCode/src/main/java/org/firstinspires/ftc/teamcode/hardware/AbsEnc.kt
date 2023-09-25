package org.firstinspires.ftc.teamcode.hardware

import com.qualcomm.robotcore.hardware.AnalogInput
import org.firstinspires.ftc.teamcode.utils.RobotFuncs
import kotlin.math.PI

class AbsEnc(name: String) {
    private val enc: AnalogInput

    init {
        enc = RobotFuncs.hardwareMap.get(AnalogInput::class.java, name)
    }

    val pos: Double
        get() {
            return (enc.voltage / enc.maxVoltage) * PI * 2
        }
}