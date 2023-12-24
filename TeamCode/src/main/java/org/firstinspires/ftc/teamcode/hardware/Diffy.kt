package org.firstinspires.ftc.teamcode.hardware

import org.firstinspires.ftc.teamcode.utils.RobotVars.DIFDOWN
import org.firstinspires.ftc.teamcode.utils.RobotVars.DIFFUP
import org.firstinspires.ftc.teamcode.utils.RobotVars.DIFLOFF

class Diffy(name: String) {
    //val LE = AbsEnc(name + "LE", OFFDL)
    val LS = MServo(name + "LS", true, DIFDOWN - DIFFUP / 2.0)
    //val RE = AbsEnc(name + "RE", OFFDR)
    val RS = MServo(name + "RS", false, DIFDOWN + DIFFUP / 2.0)

    var targetDiff = 0.0
        set(v) {
            LS.position = targetPos - targetDiff / 2.0 + DIFLOFF
            RS.position = targetPos + targetDiff / 2.0
            field = v
        }

    var targetPos = 0.0
        set(v) {
            LS.position = targetPos - targetDiff / 2.0 + DIFLOFF
            RS.position = targetPos + targetDiff / 2.0
            field = v
        }
}