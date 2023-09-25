package org.firstinspires.ftc.teamcode.hardware

import com.acmerobotics.dashboard.config.Config
import com.qualcomm.robotcore.hardware.CRServo
import com.qualcomm.robotcore.util.ElapsedTime
import org.firstinspires.ftc.teamcode.utils.RobotFuncs
import org.firstinspires.ftc.teamcode.utils.Util.angDiff

class CServo(sname: String, ename: String) {
    private val s: CRServo
    private val e: AbsEnc

    private val pid: Thread
    private var pidRunning: Boolean = false

    @Config
    companion object {
        @JvmField
        var p = 0.0
        var i = 0.0
        var d = 0.0
        var f = 0.0
    }

    init {
        s = RobotFuncs.hardwareMap.get(CRServo::class.java, sname)
        e = AbsEnc(ename)

        pid = Thread {
            var err: Double
            var der: Double
            var cp: Double

            var lastErr = 0.0
            var int = 0.0

            val timer = ElapsedTime()
            timer.reset()
            while (pidRunning) {
                cp = e.pos
                err = angDiff(pt, cp)
                der = angDiff(err, lastErr) / timer.seconds()
                int += (err * timer.seconds())
                lastErr = err

                s.power = f + err * p + der * d + int * i
            }
        }
    }

    var pt: Double = 0.0

    fun initPid() {
        pidRunning = true
        pid.start()
    }

    fun joinPid() {
        pidRunning = false
        pid.join()
    }
}