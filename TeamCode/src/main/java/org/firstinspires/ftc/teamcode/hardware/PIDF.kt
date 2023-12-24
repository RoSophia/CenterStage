package org.firstinspires.ftc.teamcode.hardware

import com.acmerobotics.dashboard.telemetry.TelemetryPacket
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import com.qualcomm.robotcore.hardware.DcMotor
import com.qualcomm.robotcore.hardware.DcMotorEx
import com.qualcomm.robotcore.util.ElapsedTime
import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit
import org.firstinspires.ftc.teamcode.utils.RobotFuncs.dashboard
import org.firstinspires.ftc.teamcode.utils.RobotFuncs.log
import org.firstinspires.ftc.teamcode.utils.RobotVars.RBOT_POS
import org.firstinspires.ftc.teamcode.utils.RobotVars.USE_TELE
import org.firstinspires.ftc.teamcode.utils.RobotVars.pcoef
import kotlin.math.abs


class PIDF(private val motA: Motor, private val motB: Motor?, private val name: String, private val lom: LinearOpMode,
           private var p: Double, private var i: Double, private var d: Double, private var f: Double, private var b: Double, private var md: Double) {
    companion object {
        private const val CORRECTION = 0.13
        private const val MAX_CURRENT_DRAW = 7000
        private const val MAX_OVERCURRENT_TIME = 2.0
        private const val A = 0.1
        private const val B = 2.9
    }

    var shouldClose = false
    var use = true
    var DUR = 1.0

    fun update_pid(p: Double, d: Double, i: Double, f: Double, b: Double) {
        this.p = p
        this.d = d
        this.i = i
        this.f = f
        this.b = b
    }

    private var target = 0
    private var ctarg = 0
    private var ltarg = 0

    private var ttim = ElapsedTime(0)

    fun set_target(targ: Int, tim: Double) { /// Start a new movement from `target` to `targ` in `tim` time. The actual calculations are done in `updt()`
        ltarg = target
        target = targ
        DUR = tim
        ttim.reset()
        integralSum = 0.0
    }

    fun updt() {
        val x: Double = if (DUR > 0.0001) {
            ttim.seconds() * (1 / DUR) /// Rescale the elapsed time to [0, 1]
        } else {
            314.0
        }
        log(name + "CurX", x)
        ctarg = if (x <= 1) { /// If we have not yet reached the end of the movement, set the current target as `a*x*(1-x)^3 + b*(1-x)*x + x^3` from `ltarg` to `targ`
            ltarg + ((A * x * (1 - x) * (1 - x) * (1 - x) + B * (1 - x) * x + x * x * x) * (target - ltarg)).toInt()
        } else { /// We have already reached the destination and need not update any further
            target
        }
    }

    private var error = 0.0
    private var derivate = 0.0
    private var lastError = 0.0
    private var integralSum = 0.0
    private var lastTarget = 0.0

    fun sign(v: Double): Double {
        return if (v > 0.0) {
            1.0
        } else if (v < 0.0) {
            -1.0
        } else {
            0.0
        }
    }

    fun run() {
        var cp: Int
        var cpb = 0
        val timer2 = ElapsedTime(0)
        lastTarget = target.toDouble()
        ctarg = target
        var outp = 0.0
        val timer = ElapsedTime(0)
        var lf = false
        val MOTATIMER = ElapsedTime(0)
        MOTATIMER.reset()
        while (!shouldClose && !lom.isStopRequested && lom.opModeIsActive()) {
            cp = motA.currentPosition
            if (motB != null) {
                cpb = motB.currentPosition
            }
            if (USE_TELE) {
                val pack = TelemetryPacket()
                pack.put(name + "CycleTimeArm", timer2.milliseconds())
                timer2.reset()
                pack.put(name + "Target", target)
                pack.put(name + "lTarg", ltarg)
                pack.put(name + "cTarg", ctarg)
                pack.put(name + "ttim", ttim.seconds())
                pack.put(name + "Dur", DUR)
                pack.put(name + "CA", cp)
                if (motB != null) {
                    pack.put(name + "CB", cpb)
                }
                pack.put(name + "Power", outp)
                pack.put(name + "Error", error)
                pack.put(name + "Derivate", derivate)
                pack.put(name + "Isum", integralSum)
                pack.put(name + "LF", lf)
                pack.put(name + "use", use)
                pack.put(name + "PID_p", p)
                pack.put(name + "PID_i", i)
                pack.put(name + "PID_d", d)
                pack.put(name + "PID_f", f)
                dashboard.sendTelemetryPacket(pack)
            }
            updt()
            if (use) {
                if (lastTarget != target.toDouble()) {
                    lastTarget = target.toDouble()
                }
                error = (ctarg - cp).toDouble()
                derivate = (error - lastError) / timer.seconds()
                integralSum += error * timer.seconds()
                outp = p * error + d * derivate + i * integralSum + f

                if (ctarg == target) {
                    if (abs(ctarg - cp) < md) {
                        if (ctarg <= RBOT_POS) {
                            outp = 0.0
                        } else {
                            if (abs(ctarg - cp) < md / 4) {
                                lf = true
                            }
                            outp = if (!lf) {
                                CORRECTION * sign((ctarg - cp).toDouble())
                            } else {
                                f
                            }
                        }
                    } else {
                        lf = false
                    }
                }

                /// KILL MYSELF
                if (target == RBOT_POS && cp > 15) {
                    outp = -1.0
                }

                /// KRILL MYSELF
                if (motA.current < MAX_CURRENT_DRAW || (if (motB != null) motB.current < MAX_CURRENT_DRAW else true)) {
                    MOTATIMER.reset()
                } else if (MOTATIMER.seconds() > MAX_OVERCURRENT_TIME) {
                    outp = 0.0
                    ctarg = cp
                    target = ctarg
                }
                if (!shouldClose && !lom.isStopRequested && lom.opModeIsActive()) { /// `lom` here is used to prevent powering the motor after the OpMode stopped.
                    motA.power = outp * pcoef
                    if (motB != null) {
                        var bdif = (cp - cpb) * b
                        if (abs(cp - cpb) < 4 || ctarg < 15) {
                            bdif = 0.0
                        }
                        motB.power = (outp + bdif) * pcoef
                    }
                } else {
                    motA.power = 0.0
                    motB?.power = 0.0
                }
                lastError = error
                timer.reset()
            } else {
                integralSum = 0.0
                lastError = integralSum
                derivate = lastError
                error = derivate
            }

            try {
                Thread.sleep(10)
            } catch (e: InterruptedException) {
                e.printStackTrace()
            }
        }
        motA.power = 0.0
        motB?.power = 0.0
    }

}