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
    val LS = MServo(name + "LS", true)
    val RS = MServo(name + "RS", false)

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

    var lcp = -100.0
    var targetPos = 0.0
        set(v) {
            if (v != field) {
                if (slides.pos < 0.5 && targetPos > 0.5) {
                    clown.position = GhearaSINCHIS
                }
                if (slides.pos > 0.5 && targetPos < 0.5 && epsEq(clown.position, GhearaSDESCHIS)) {
                    lcp = GhearaSDESCHIS
                    log("DiffyUpClose", slides.pos)
                    clown.position = GhearaSINCHIS
                }
                if (epsEq(lcp, GhearaSDESCHIS) && slides.pos < 0.5) {
                    clown.position = lcp
                    log("DiffyDownClose", slides.pos)
                    lcp = -100.0
                }
                field = v
                updateTarget()
            }
        }
}