package org.firstinspires.ftc.teamcode.hardware

import com.qualcomm.robotcore.hardware.DcMotor
import org.firstinspires.ftc.teamcode.utils.NanoClock
import org.firstinspires.ftc.teamcode.utils.RobotFuncs.hardwareMap

class Encoder(val name: String) {
    val clock = NanoClock()
    var direction = 1
    var lastPosition = 0
    var m = hardwareMap.get(DcMotor::class.java, name)

    var intPos: Int = 0

    var pos: Int = 0
    var vel: Double = 0.0

    fun upd() {
        val cp = m.currentPosition * direction
        vel = (cp - lastPosition).toDouble() / clock.seconds()
        lastPosition = pos
        pos = cp
        clock.reset()
    }
}
