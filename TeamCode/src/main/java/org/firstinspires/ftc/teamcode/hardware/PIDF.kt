package org.firstinspires.ftc.teamcode.hardware

import com.acmerobotics.dashboard.telemetry.TelemetryPacket
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import com.qualcomm.robotcore.hardware.DcMotor
import com.qualcomm.robotcore.hardware.DcMotorEx
import com.qualcomm.robotcore.util.ElapsedTime
import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit
import org.firstinspires.ftc.teamcode.utils.PID
import org.firstinspires.ftc.teamcode.utils.RobotFuncs.dashboard
import org.firstinspires.ftc.teamcode.utils.RobotFuncs.log
import org.firstinspires.ftc.teamcode.utils.RobotFuncs.logs
import org.firstinspires.ftc.teamcode.utils.RobotVars.RBOT_POS
import org.firstinspires.ftc.teamcode.utils.RobotVars.RidicarePid
import org.firstinspires.ftc.teamcode.utils.RobotVars.RidicarePower
import org.firstinspires.ftc.teamcode.utils.RobotVars.USE_TELE
import kotlin.math.abs
import kotlin.math.sign


class PIDF(private val motA: Motor, private val motB: Motor, private val enc: Encoder, private var md: Double) {
    companion object {
        private const val MAX_CURRENT_DRAW = 7000
        private const val MAX_OVERCURRENT_TIME = 2.0
        private const val A = 0.1
        private const val B = 2.9
    }

    var DUR = 1.0

    private var target = 0
    private var ctarg = 0
    private var ltarg = 0
    var use = true

    private var ttim = ElapsedTime(0)

    fun set_target(targ: Int, tim: Double) { /// Start a new movement from `target` to `targ` in `tim` time. The actual calculations are done in `updt()`
        ltarg = target
        target = targ
        DUR = tim
        ttim.reset()
    }

    fun updt() {
        val x: Double = if (DUR > 0.0001) {
            ttim.seconds() * (1 / DUR) /// Rescale the elapsed time to [0, 1]
        } else {
            314.0
        }
        ctarg = if (x <= 1) { /// If we have not yet reached the end of the movement, set the current target as `a*x*(1-x)^3 + b*(1-x)*x + x^3` from `ltarg` to `targ`
            ltarg + ((A * x * (1 - x) * (1 - x) * (1 - x) + B * (1 - x) * x + x * x * x) * (target - ltarg)).toInt()
        } else { /// We have already reached the destination and need not update any further
            target
        }
    }

    val MOTATIMER = ElapsedTime(0)

    val motAP = PID(RidicarePid)
    val motBP = PID(RidicarePid)

    fun update(): Double {
        if (use) {
            val cp: Int = enc.pos
            updt()
            if (target == RBOT_POS && cp > 10) {
                return RidicarePower
            }
            var outp = motAP.update((ctarg - cp).toDouble())

            if (abs(ctarg - cp) < md) {
                if (ctarg <= RBOT_POS) {
                    outp = 0.0
                }
            }

            /* TODO: Overcurrent
            if (motA.current < MAX_CURRENT_DRAW && motB.current < MAX_CURRENT_DRAW) {
                MOTATIMER.reset()
            }

            if (MOTATIMER.seconds() > MAX_OVERCURRENT_TIME) {
                outp = 0.0
                ctarg = cp
                target = ctarg
            }
             */
            if (cp < 5 && ctarg == RBOT_POS) {
                outp = 0.0
            }
            /*
            logs("RidicareTarPos", ctarg)
            logs("RidicareOutp", outp)*/
            return outp
        } else {
            motAP.reset()
            return 0.0
        }
    }

}