package org.firstinspires.ftc.teamcode.hardware

import com.acmerobotics.dashboard.FtcDashboard
import com.acmerobotics.dashboard.telemetry.TelemetryPacket
import com.qualcomm.hardware.bosch.BNO055IMU
import com.qualcomm.robotcore.util.ElapsedTime
import org.firstinspires.ftc.teamcode.utils.RobotFuncs
import org.firstinspires.ftc.teamcode.utils.RobotFuncs.KILLALL
import org.firstinspires.ftc.teamcode.utils.RobotVars.TimmyLoopTime
import org.firstinspires.ftc.teamcode.utils.RobotVars.TimmyCurOff

class Timmy(val name: String) {
    private val imu: BNO055IMU = RobotFuncs.hardwareMap.get(BNO055IMU::class.java, name)

    private val t: Thread
    var trunning: Boolean = false
    private var initialized: Boolean = false

    var yaw: Double = 0.0
    var yawVel: Double = 0.0
    var localizerAccessed = false

    val ep = ElapsedTime()

    var lep: Double = 0.0

    init {
        val parameters = BNO055IMU.Parameters()
        parameters.angleUnit = BNO055IMU.AngleUnit.RADIANS
        //parameters.mode = BNO055IMU.SensorMode.COMPASS
        imu.initialize(parameters)
        initialized = true
        ep.reset()

        t = Thread {
            val ep = ElapsedTime()
            ep.reset()
            while (trunning && !KILLALL) {
                val fixed = imu.angularOrientation
                val y = fixed.firstAngle.toDouble() - TimmyCurOff
                yaw = y
                /*val tp = TelemetryPacket()
                tp.put("TYAW", yaw)
                FtcDashboard.getInstance().sendTelemetryPacket(tp)*/
                localizerAccessed = false
                //yawVel = imu.angularVelocity.xRotationRate.toDouble()
                TimmyLoopTime = ep.seconds()
                lep = ep.seconds()
                ep.reset()
            }
        }
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