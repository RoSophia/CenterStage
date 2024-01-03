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

    private val enc: AnalogInput = RobotFuncs.hardwareMap.get(AnalogInput::class.java, name)

    private val timer = ElapsedTime()

    private var cnrpos = 0

    init {
        timer.reset()
        val g = nrRots[name]
        if (g != null) {
            cnrpos = g
        }
    }

    private val maxVoltage = 3.305
    private val angPer01 = (PI * 2 * gearr)

    private val de = ArrayDeque<tmp>()

    val angle: Double
        get() {
        val cv = enc.voltage / maxVoltage
        de.addLast(tmp(timer.seconds(), cv))

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

        if (cv > 0.8 && dl.pos < 0.2) {
            while (de.first().pos < 0.2) {
                de.removeFirst()
            }
            --cnrpos
            nrRots[name] = cnrpos
        } else if (cv < 0.2 && dl.pos > 0.8) {
            while (de.first().pos > 0.8) {
                de.removeFirst()
            }
            ++cnrpos
            nrRots[name] = cnrpos
        }

        logs("AbsEnc_${name}_Volt", cv)
        logs("AbsEnc_${name}_nrRot", cnrpos)
        return (cv + cnrpos) * angPer01 + off
    }

    val pos: Double
        get() {
            return angNorm(angle)
        }
}