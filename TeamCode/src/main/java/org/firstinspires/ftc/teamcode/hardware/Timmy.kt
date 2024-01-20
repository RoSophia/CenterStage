package org.firstinspires.ftc.teamcode.hardware

import com.qualcomm.hardware.bosch.BNO055IMU
import com.qualcomm.robotcore.util.ElapsedTime
import kotlinx.coroutines.sync.Mutex
import org.firstinspires.ftc.robotcore.external.navigation.Orientation
import org.firstinspires.ftc.teamcode.utils.RobotFuncs
import org.firstinspires.ftc.teamcode.utils.RobotFuncs.KILLALL
import org.firstinspires.ftc.teamcode.utils.RobotFuncs.logs
import org.firstinspires.ftc.teamcode.utils.RobotVars.AvionDeschis
import org.firstinspires.ftc.teamcode.utils.RobotVars.AvionInchis
import org.firstinspires.ftc.teamcode.utils.RobotVars.TIMMYA

class Timmy(val name: String) {
    private val imu: BNO055IMU = RobotFuncs.hardwareMap.get(BNO055IMU::class.java, name)
    private val avion: MServo = MServo("Pewpew", AvionInchis)

    private val t: Thread
    private var trunning: Boolean = false
    private var initialized: Boolean = false

    private fun fixAngOrientation(i: Orientation): Orientation {
        return i
    }

    var yaw: Double = 0.0
    var yawVel: Double = 0.0

    val ep = ElapsedTime()

    var lep: Double = 0.0

    fun openAvion() {
        avion.position = AvionDeschis
    }

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
                val fixed = fixAngOrientation(imu.angularOrientation)
                val y = fixed.firstAngle.toDouble()
                yaw = y
                yawVel = imu.angularVelocity.xRotationRate.toDouble()
                TIMMYA = ep.seconds()
                lep = ep.seconds()
                ep.reset()
            }
        }
    }

    fun update() {
        val fixed = fixAngOrientation(imu.angularOrientation)
        val y = fixed.firstAngle.toDouble()
        yaw = y
        yawVel = imu.angularVelocity.xRotationRate.toDouble()
        logs("IMU_yaw", y)
        logs("IMU_CycleTime", ep.seconds())
        logs("IMU_ango", fixed)
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
        logs("Timmy_${name}_Status", "Close")
    }


}