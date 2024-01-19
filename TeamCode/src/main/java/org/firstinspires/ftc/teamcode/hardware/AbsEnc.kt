package org.firstinspires.ftc.teamcode.hardware

import com.acmerobotics.dashboard.telemetry.TelemetryPacket
import com.qualcomm.robotcore.hardware.AnalogInput
import com.qualcomm.robotcore.util.ElapsedTime
import org.firstinspires.ftc.teamcode.utils.RobotFuncs
import org.firstinspires.ftc.teamcode.utils.RobotFuncs.dashboard
import org.firstinspires.ftc.teamcode.utils.RobotFuncs.log
import org.firstinspires.ftc.teamcode.utils.RobotFuncs.logs
import org.firstinspires.ftc.teamcode.utils.RobotVars.nrRots
import org.firstinspires.ftc.teamcode.utils.Util.angNorm
import kotlin.math.PI

class tmp(val time: Double, val pos: Double)

class AbsEnc(private val name: String, private val off: Double, gearr: Double) {
    constructor(name: String, off: Double) : this(name, off, 1.0)

    val enc: AnalogInput = RobotFuncs.hardwareMap.get(AnalogInput::class.java, name)

    private val timer = ElapsedTime()

    private var cnrpos = 0

    private val de = ArrayDeque<tmp>()

    private val maxVoltage = 3.305
    private val angPer01 = (PI * 2 * gearr)

    init {
        timer.reset()
        val g = nrRots[name]
        val cv = enc.voltage / maxVoltage
        val cvo = angNorm(cv * angPer01 + off)
        if (name == "DifLE" || name == "DifRE") {
            if (cvo > 3.0) {
                cnrpos = -1
            }
            de.addLast(tmp(timer.seconds(), cvo))
        }
        if (g != null) {
            cnrpos = g
        }
    }

    val angle: Double
        get() {
            val cv = enc.voltage / maxVoltage
            val cvo = angNorm(cv * angPer01 + off)
            if (name == "DifLE" || name == "DifRE") {
                de.addLast(tmp(timer.seconds(), cvo))
            } else {
                de.addLast(tmp(timer.seconds(), cv))
            }
            val ts = timer.seconds()
            var dl: tmp = de.first()

            while (true) {
                if (ts - de.first().time > 0.1 && de.size > 1) {
                    dl = de.first()
                    de.removeFirst()
                } else {
                    break
                }
            }

            if (name == "DifLE" || name == "DifRE") {
                if (cvo > 5 && dl.pos < 1) {
                    while (de.size > 0 && de.first().pos < 1) {
                        de.removeFirst()
                    }
                    --cnrpos
                    nrRots[name] = cnrpos
                } else if (cvo < 1 && dl.pos > 5) {
                    while (de.size > 0 && de.first().pos > 5) {
                        de.removeFirst()
                    }
                    ++cnrpos
                    nrRots[name] = cnrpos
                }
                logs("AbsEnc_${name}_cvo", cvo)
                logs("AbsEnc_${name}_Volt", cv)
                logs("AbsEnc_${name}_nrRot", cnrpos)
                logs("AbsEnc_${name}_output", cvo + cnrpos * angPer01)
                return cvo + cnrpos * angPer01
            } else {
                if (cv > 0.8 && dl.pos < 0.2) {
                    while (de.size > 0 && de.first().pos < 0.2) {
                        de.removeFirst()
                    }
                    --cnrpos
                    nrRots[name] = cnrpos
                } else if (cv < 0.2 && dl.pos > 0.8) {
                    while (de.size > 0 && de.first().pos > 0.8) {
                        de.removeFirst()
                    }
                    ++cnrpos
                    nrRots[name] = cnrpos
                }
                logs("AbsEnc_${name}_cvo", cvo)
                logs("AbsEnc_${name}_Volt", cv)
                logs("AbsEnc_${name}_nrRot", cnrpos)
                logs("AbsEnc_${name}_output", cvo + cnrpos * angPer01)
                return (cv + cnrpos) * angPer01 + off
            }
        }

    val angn: Double
        get() {
            return angNorm(angle)
        }
}