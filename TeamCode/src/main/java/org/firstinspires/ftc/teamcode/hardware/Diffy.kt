package org.firstinspires.ftc.teamcode.hardware

import com.qualcomm.robotcore.util.ElapsedTime
import org.firstinspires.ftc.teamcode.utils.RobotFuncs.log
import org.firstinspires.ftc.teamcode.utils.RobotVars.DIFGRAT
import org.firstinspires.ftc.teamcode.utils.RobotVars.DIFLOFF
import org.firstinspires.ftc.teamcode.utils.RobotVars.DIFROFF
import org.firstinspires.ftc.teamcode.utils.RobotVars.DiffyPid
import org.firstinspires.ftc.teamcode.utils.RobotVars.USE_DIFFY

class Diffy(name: String) {
    val LS = CServo(name + "L", DIFLOFF, DIFGRAT, false, DiffyPid)
    val RS = CServo(name + "R", DIFROFF, DIFGRAT, false, DiffyPid)

    fun update() {
        if (USE_DIFFY) {
            RS.update()
            LS.update()
        }
    }

    fun init() {
        if (USE_DIFFY) {
            LS.pt = 0.0
            RS.pt = 0.0
            LS.init()
            RS.init()
        }
    }

    fun close() {
        if (USE_DIFFY) {
            LS.close()
            RS.close()
        }
    }

    /// Since the left servo is not flipped the PIDF coefficients have to be negative,
    /// and also the angle and Diff and Pos terms be reversed
    fun updateTarget() {
        LS.pt = targetDiff - targetPos / 2.0
        RS.pt = targetDiff + targetPos / 2.0
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