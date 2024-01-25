package org.firstinspires.ftc.teamcode.hardware

import org.firstinspires.ftc.teamcode.utils.RobotFuncs.clown
import org.firstinspires.ftc.teamcode.utils.RobotFuncs.log
import org.firstinspires.ftc.teamcode.utils.RobotFuncs.slides
import org.firstinspires.ftc.teamcode.utils.RobotVars.DLOFF
import org.firstinspires.ftc.teamcode.utils.RobotVars.DROFF
import org.firstinspires.ftc.teamcode.utils.RobotVars.GhearaSDESCHIS
import org.firstinspires.ftc.teamcode.utils.RobotVars.GhearaSINCHIS
import org.firstinspires.ftc.teamcode.utils.RobotVars.USE_DIFFY
import org.firstinspires.ftc.teamcode.utils.Util.epsEq

class Diffy(name: String) {
    val LS = MServo(name + "LS", true, 0.3)
    val RS = MServo(name + "RS", false, 0.3)

    fun updateTarget() {
        if (USE_DIFFY) {
            RS.position = (targetPos / 2.0 + targetDiff) + DROFF
            LS.position = (targetPos / 2.0 - targetDiff) + DLOFF
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