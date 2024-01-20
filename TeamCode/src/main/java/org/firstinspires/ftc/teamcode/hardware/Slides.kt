package org.firstinspires.ftc.teamcode.hardware

import com.qualcomm.robotcore.util.ElapsedTime
import org.firstinspires.ftc.teamcode.utils.RobotFuncs.log
import org.firstinspires.ftc.teamcode.utils.RobotFuncs.logs
import org.firstinspires.ftc.teamcode.utils.RobotVars.*

class Slides {
    val enc = Encoder(RidicareEncoderName, RidicareEncoderDir)
    val l = Motor("RidL", encoder = false, rev = false, overdrive = true)
    val r = Motor("RidR", encoder = false, rev = true, overdrive = true)
    val p = PIDF(l, r, enc, 2.0)

    val pos: Int
        get() = enc.pos

    fun setTarget(t: Int) {
        p.set_target(t, RidicareTime)
    }

    var power = 0.0
        set(v) {
            field = if (pos < 10 && v < 0.0) {
                log("StoppingPower", "DOWN:$v")
                0.0
            } else if (pos > RTOP_POS + RidicareLeeway && v > 0.0) {
                log("StoppingPower", "UP:$v")
                0.0
            } else {
                v
            }
            tryMove = true
        }

    private var tryMove = false

    val ep = ElapsedTime()

    fun update() {
        if (USE_RIDICARE) {
            logs("RidicareCurPos", pos)
            logs("SlidesTryMove", tryMove)
            logs("SlidesTargetPower", power)
            if (!tryMove) {
                p.use = true
                power = p.update()
            } else {
                p.use = false
                p.update()
                ep.reset()
            }
            if (ep.seconds() < RidicareMaxTime) {
                p.set_target(pos, 0.0)
            }
            if (tryMove) {
                r.power = power
                l.power = power
            }
            tryMove = false
        } else {
            r.power = 0.0
            l.power = 0.0
        }
    }
}