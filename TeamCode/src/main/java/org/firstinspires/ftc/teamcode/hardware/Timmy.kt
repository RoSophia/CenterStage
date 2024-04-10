package org.firstinspires.ftc.teamcode.hardware

import com.qualcomm.hardware.bosch.BNO055IMU
import com.qualcomm.robotcore.util.ElapsedTime
import org.firstinspires.ftc.teamcode.utils.RobotFuncs
import org.firstinspires.ftc.teamcode.utils.RobotFuncs.KILLALL
import org.firstinspires.ftc.teamcode.utils.RobotFuncs.log
import org.firstinspires.ftc.teamcode.utils.RobotFuncs.logs
import org.firstinspires.ftc.teamcode.utils.RobotVars.TimmyCurOff
import org.firstinspires.ftc.teamcode.utils.Util.angNorm

class Timmy(val name: String) {
    private val imu: BNO055IMU = RobotFuncs.hardwareMap.get(BNO055IMU::class.java, name)

    var trunning: Boolean = false
    var initialized: Boolean = false

    val yaw: Double
        get() = angNorm(_yaw + TimmyCurOff)
    private var _yaw = 0.0
    var yawVel: Double = 0.0
    var localizerAccessed = false

    fun resetYaw(d: Double) {
        logs("ActualYaw", _yaw)
        logs("OldYaw", yaw)
        TimmyCurOff = -_yaw + d
        logs("NewYaw", yaw)
    }

    val ep = ElapsedTime()

    private class Timtim(val t: Timmy) : Runnable {
        override fun run() {
            val ep = ElapsedTime()
            ep.reset()
            while (t.trunning && !KILLALL) {
                val fixed = t.imu.angularOrientation
                val y = fixed.firstAngle.toDouble()
                t._yaw = y
                t.localizerAccessed = false
                ep.reset()
            }
        }
    }

    var t = Thread()
    fun init() {
        val parameters = BNO055IMU.Parameters()
        parameters.angleUnit = BNO055IMU.AngleUnit.RADIANS
        imu.initialize(parameters)
        initialized = true
        ep.reset()

        t = Thread(Timtim(this))
        t.setUncaughtExceptionHandler { th, er -> RobotFuncs.log("GOT ERR TIMMY ${th.id}", er.stackTraceToString()) }
        //t.start()
    }

    fun initThread() {
        if (!trunning) {
            trunning = true
            t.start()
        }
    }

    fun closeThread() {
        if (trunning) {
            trunning = false
            t.join()
        }
    }

    fun close() {
        closeThread()
        if (initialized) {
            imu.close()
            initialized = false
        }
    }
}
