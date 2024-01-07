package org.firstinspires.ftc.teamcode.hardware

import com.acmerobotics.dashboard.telemetry.TelemetryPacket
import com.outoftheboxrobotics.photoncore.hardware.servo.PhotonCRServo
import com.qualcomm.robotcore.hardware.CRServo
import com.qualcomm.robotcore.hardware.DcMotorSimple
import com.qualcomm.robotcore.util.ElapsedTime
import org.firstinspires.ftc.teamcode.utils.RobotFuncs
import org.firstinspires.ftc.teamcode.utils.RobotFuncs.KILLALL
import org.firstinspires.ftc.teamcode.utils.RobotFuncs.dashboard
import org.firstinspires.ftc.teamcode.utils.RobotFuncs.log
import org.firstinspires.ftc.teamcode.utils.RobotFuncs.logs
import org.firstinspires.ftc.teamcode.utils.RobotVars.LOG_STATUS
import org.firstinspires.ftc.teamcode.utils.RobotVars.USE_TELE
import org.firstinspires.ftc.teamcode.utils.Util.angDiff
import org.firstinspires.ftc.teamcode.utils.Util.angNorm
import kotlin.math.abs
import kotlin.math.sign

class PIDFC(@JvmField var p: Double, @JvmField var i: Double, @JvmField var d: Double, @JvmField var f: Double)

class CServo(val name: String, eoff: Double, gearr: Double, private val can360: Boolean, var pd1: PIDFC, var pd2: PIDFC) {
    constructor(name: String, eoff: Double, gearr: Double, can360: Boolean, pd1: PIDFC) : this(name, eoff, gearr, can360, pd1, pd1)
    constructor(name: String, eoff: Double, gearr: Double, can360: Boolean) : this(name, eoff, gearr, can360, PIDFC(0.0, 0.0, 0.0, 0.0))
    constructor(name: String, eoff: Double, gearr: Double) : this(name, eoff, gearr, true)

    private val s = MCRServo(name + "S")
    val e = AbsEnc(name + "E", eoff, gearr)

    //private val pid: Thread
    private var pidRunning: Boolean = false

    /*
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
                cp = e.angle
                err = if (can360) {
                    angDiff(pt, cp)
                } else {
                    pt - cp
                }
                if (abs(err) < 0.02) {
                    err = 0.0
                    lastErr = 0.0
                }
                der = if (can360) {
                    angDiff(err, lastErr) / timer.seconds()
                } else {
                    (err - lastErr) / timer.seconds()
                }
                int += (err * timer.seconds())
                lastErr = err

                if (can360 && err > 3.0) {
                    err = 0.0
                    der = 0.0
                    int = 0.0
                }

                if (err >= 0.0) {
                    s.power = sign(err) * pd1.f + err * pd1.p + der * pd1.d + int * pd1.i
                } else {
                    s.power = sign(err) * pd2.f + err * pd2.p + der * pd2.d + int * pd2.i
                }

                if (USE_TELE) {
                    val tp = TelemetryPacket()
                    if (LOG_STATUS) {
                        logs("CServo_${name}_pidf1", "${pd1.f} - ${pd1.p} - ${pd1.d} - ${pd1.i}")
                        logs("CServo_${name}_pidf2", "${pd2.f} - ${pd2.p} - ${pd2.d} - ${pd2.i}")
                        logs("CServo_${name}_der", der)
                        logs("CServo_${name}_Pow", s.power)
                        logs("CServo_${name}_Timer", timer.seconds())
                    }
                    log("CServo_${name}_Enc", angNorm(cp))
                    log("CServo_${name}_Tar", angNorm(pt))
                    dashboard.sendTelemetryPacket(tp)
                }
                timer.reset()
            }
        }
    }
     */

    var err: Double = 0.0
    var der: Double = 0.0
    var cp: Double = 0.0
    val timer = ElapsedTime()

    var lastErr = 0.0
    var int = 0.0
    fun update() {
        cp = e.angle
        err = if (can360) {
            angDiff(pt, cp)
        } else {
            pt - cp
        }
        if (abs(err) < 0.02) {
            err = 0.0
            lastErr = 0.0
        }
        der = if (can360) {
            angDiff(err, lastErr) / timer.seconds()
        } else {
            (err - lastErr) / timer.seconds()
        }
        int += (err * timer.seconds())
        lastErr = err

        if (can360 && err > 3.0) {
            err = 0.0
            der = 0.0
            int = 0.0
        }

        if (err >= 0.0) {
            s.power = sign(err) * pd1.f + err * pd1.p + der * pd1.d + int * pd1.i
        } else {
            s.power = sign(err) * pd2.f + err * pd2.p + der * pd2.d + int * pd2.i
        }

        logs("CServo_${name}_pidf1", "${pd1.f} - ${pd1.p} - ${pd1.d} - ${pd1.i}")
        logs("CServo_${name}_pidf2", "${pd2.f} - ${pd2.p} - ${pd2.d} - ${pd2.i}")
        logs("CServo_${name}_der", der)
        logs("CServo_${name}_Pow", s.power)
        logs("CServo_${name}_Timer", timer.seconds())
        log("CServo_${name}_Enc", angNorm(cp))
        log("CServo_${name}_Tar", angNorm(pt))
        timer.reset()
    }

    var pt: Double = 0.0

    fun init() {
        //pidRunning = true
        //pid.start()
        timer.reset()
        //logs("CServo_${name}_PID_Status", "Init")
    }

    fun close() {
        /*
        pidRunning = false
        pid.join()
        logs("CServo_${name}_PID_Status", "Close")*/
    }
}
