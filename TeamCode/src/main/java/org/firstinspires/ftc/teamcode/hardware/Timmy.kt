package org.firstinspires.ftc.teamcode.hardware

import com.qualcomm.hardware.bosch.BNO055IMU
import com.qualcomm.robotcore.util.ElapsedTime
import org.firstinspires.ftc.teamcode.utils.RobotFuncs
import org.firstinspires.ftc.teamcode.utils.RobotFuncs.KILLALL
import org.firstinspires.ftc.teamcode.utils.RobotVars.TIMMYA
import org.firstinspires.ftc.teamcode.utils.RobotVars.TIMMYOFF

class Timmy(val name: String) {
    private val imu: BNO055IMU = RobotFuncs.hardwareMap.get(BNO055IMU::class.java, name)

    private val t: Thread
    private var trunning: Boolean = false
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
                val y = fixed.firstAngle.toDouble() - TIMMYOFF
                yaw = y
                localizerAccessed = false
                yawVel = imu.angularVelocity.xRotationRate.toDouble()
                TIMMYA = ep.seconds()
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