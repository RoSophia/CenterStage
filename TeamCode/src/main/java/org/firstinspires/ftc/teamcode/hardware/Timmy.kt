package org.firstinspires.ftc.teamcode.hardware

import com.acmerobotics.dashboard.FtcDashboard
import com.acmerobotics.dashboard.telemetry.TelemetryPacket
import com.qualcomm.hardware.bosch.BNO055IMU
import com.qualcomm.hardware.bosch.BNO055IMU.CalibrationData
import com.qualcomm.robotcore.util.ElapsedTime
import org.firstinspires.ftc.teamcode.utils.RobotFuncs
import org.firstinspires.ftc.teamcode.utils.RobotFuncs.KILLALL
import org.firstinspires.ftc.teamcode.utils.RobotFuncs.log
import org.firstinspires.ftc.teamcode.utils.RobotVars.TimmyCurOff
import org.firstinspires.ftc.teamcode.utils.RobotVars.TimmyLoopTime
import org.firstinspires.ftc.teamcode.utils.TrajectorySequence

class Timmy(val name: String) {
    private val imu: BNO055IMU = RobotFuncs.hardwareMap.get(BNO055IMU::class.java, name)

    var trunning: Boolean = false
    var initialized: Boolean = false

    var yaw: Double = 0.0
    var yawVel: Double = 0.0
    var localizerAccessed = false

    val ep = ElapsedTime()

    private class Timtim(val t: Timmy) : Runnable {
        override fun run() {
            val ep = ElapsedTime()
            ep.reset()
            while (t.trunning && !KILLALL) {
                val fixed = t.imu.angularOrientation
                val y = fixed.firstAngle.toDouble() - TimmyCurOff
                //log("I MEW PARAMS", t.imu.readCalibrationData().serialize())
                t.yaw = y
                t.localizerAccessed = false
                TimmyLoopTime = ep.seconds()
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