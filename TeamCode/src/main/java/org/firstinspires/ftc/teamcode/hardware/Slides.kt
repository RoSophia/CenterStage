package org.firstinspires.ftc.teamcode.hardware

import com.qualcomm.robotcore.util.ElapsedTime
import org.firstinspires.ftc.teamcode.hardware.Intakes.SFinalHang
import org.firstinspires.ftc.teamcode.hardware.Intakes.SUp
import org.firstinspires.ftc.teamcode.utils.RobotFuncs.intake
import org.firstinspires.ftc.teamcode.utils.RobotFuncs.log
import org.firstinspires.ftc.teamcode.utils.RobotFuncs.logs
import org.firstinspires.ftc.teamcode.utils.RobotVars.*
import org.firstinspires.ftc.teamcode.utils.TrajectorySequence

class Slides {
    private val enc = Encoder(RidicareEncoderName, RidicareEncoderDir)
    private val l = Motor("RidL", encoder = false, rev = false, overdrive = true)
    private val r = Motor("RidR", encoder = false, rev = true, overdrive = true)
    private val p = PIDF(l, r, enc, 2.0)

    val pos: Int
        get() = enc.pos

    fun setTarget(t: Int, time: Double) = p.set_target(t, time)
    fun setTarget(t: Int) = setTarget(t, RidicareTime)

    var power = 0.0
        set(v) {
            field = if (pos < 10 && v < 0.0) {
                logs("RidicareStoppingPower", "DOWN:$v")
                0.0
            } else if (pos > RTOP_POS + RidicareLeeway && v > 0.0) {
                logs("RidicareStoppingPower", "UP:$v")
                RidicarePid.f
            } else {
                logs("RidicareStoppingPower", "None")
                v
            }
            tryMove = true
        }

    private var tryMove = false
    var RIDICAREEEEEEEEEE = false
    private var richd = false

    val ep = ElapsedTime()

    private var pwr: Double = 0.0
        set(v) {
            l.power = v
            r.power = v
            field = v
        }

    /**
     * Eu cedez
     */
    fun youShouldHangYourselfNOW() {
        richd = false
        RIDICAREEEEEEEEEE = true
    }

    fun update() {
        if (USE_RIDICARE) {
            if (RIDICAREEEEEEEEEE) {
                val ep = enc.pos
                TrajectorySequence().sl(0.4).aa { intake.status = SFinalHang }.runAsync()
                if (ep < RidicareHaveIHangedMyself) {
                    pwr = RidicareHANGEDMYSELF
                    richd = true
                } else if (!richd) {
                    if (ep > RidicareIHaventHangedMyslef) {
                        pwr = RidicareHANGINGMYSELF
                        richd = false
                    }
                }
            } else {
                if (!tryMove) {
                    p.use = true
                    power = p.update()
                } else {
                    p.use = false
                    p.update()
                    ep.reset()
                }
                if (ep.seconds() < RidicareMaxTime) {
                    setTarget(pos, 0.0)
                }
                if (tryMove) {
                    pwr = power
                }
                tryMove = false
            }
        } else {
            pwr = 0.0
        }
    }
}