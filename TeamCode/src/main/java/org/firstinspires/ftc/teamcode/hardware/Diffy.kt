package org.firstinspires.ftc.teamcode.hardware

import org.firstinspires.ftc.teamcode.utils.RobotVars.DiffyLOff
import org.firstinspires.ftc.teamcode.utils.RobotVars.DiffyROff
import org.firstinspires.ftc.teamcode.utils.RobotVars.USE_DIFFY

class Diffy(name: String) {
    val LS = MServo(name + "LS", true, 0.3)
    val RS = MServo(name + "RS", false, 0.3)

    fun updateTarget() {
        if (USE_DIFFY) {
            RS.position = (targetPos / 2.0 + targetDiff) + DiffyROff
            LS.position = (targetPos / 2.0 - targetDiff) + DiffyLOff
        }
    }

    var targetDiff = 0.0
        set(v) {
            if (v != field) {
                field = v
                updateTarget()
            }
        }

    var targetPos = 0.0
        set(v) {
            if (v != field) {
                field = v
                updateTarget()
            }
        }
}