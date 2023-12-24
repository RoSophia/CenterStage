package org.firstinspires.ftc.teamcode.hardware

import com.acmerobotics.dashboard.config.Config
import com.acmerobotics.dashboard.telemetry.TelemetryPacket
import com.qualcomm.robotcore.hardware.CRServo
import com.qualcomm.robotcore.hardware.DcMotorSimple
import com.qualcomm.robotcore.util.ElapsedTime
import org.firstinspires.ftc.teamcode.hardware.CSP.f
import org.firstinspires.ftc.teamcode.hardware.CSP.d
import org.firstinspires.ftc.teamcode.hardware.CSP.i
import org.firstinspires.ftc.teamcode.hardware.CSP.p
import org.firstinspires.ftc.teamcode.utils.RobotFuncs
import org.firstinspires.ftc.teamcode.utils.RobotFuncs.KILLALL
import org.firstinspires.ftc.teamcode.utils.RobotFuncs.dashboard
import org.firstinspires.ftc.teamcode.utils.RobotFuncs.logs
import org.firstinspires.ftc.teamcode.utils.RobotVars.USE_TELE
import org.firstinspires.ftc.teamcode.utils.Util.angDiff
import kotlin.math.abs
import kotlin.math.sign

@Config
object CSP {
    @JvmField
    var p = 0.4

    @JvmField
    var i = 2.0

    @JvmField
    var d = 0.0004

    @JvmField
    var f = 0.04
}


class CServo(val name: String, eoff: Double, gearr: Double) {
    private val s: CRServo = RobotFuncs.hardwareMap.get(CRServo::class.java, name + "S")
    val e: AbsEnc = AbsEnc(name + "E", eoff, gearr)

    private val pid: Thread
    private var pidRunning: Boolean = false

    init {
        s.direction = DcMotorSimple.Direction.FORWARD

        pid = Thread {
            var err: Double
            var der: Double
            var cp: Double

            var lastErr = 0.0
            var int = 0.0

            val timer = ElapsedTime()
            timer.reset()
            while (pidRunning && !KILLALL) {
                cp = e.pos
                err = angDiff(pt, cp)
                if (abs(err) < 0.02) {
                    err = 0.0
                    lastErr = 0.0
                }
                der = angDiff(err, lastErr) / timer.seconds()
                int += (err * timer.seconds())
                lastErr = err

                s.power = sign(err) * f + err * p + der * d + int * i
                if (USE_TELE) {
                    val tp = TelemetryPacket()
                    tp.put("CServo_${name}_Enc", cp)
                    tp.put("CServo_${name}_der", der)
                    tp.put("CServo_${name}_Tar", pt)
                    tp.put("CServo_${name}_Pow", s.power)
                    tp.put("CServo_${name}_Timer", timer.seconds())
                    dashboard.sendTelemetryPacket(tp)
                }
                timer.reset()
            }
        }
    }

    var pt: Double = 0.0

    fun init() {
        pidRunning = true
        pid.start()
        logs("CServo_${name}_PID_Status", "Init")
    }

    fun close() {
        pidRunning = false
        pid.join()
        logs("CServo_${name}_PID_Status", "Close")
    }
}
