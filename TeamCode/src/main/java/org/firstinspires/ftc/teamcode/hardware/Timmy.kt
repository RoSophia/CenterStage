package org.firstinspires.ftc.teamcode.hardware

import com.acmerobotics.dashboard.telemetry.TelemetryPacket
import com.qualcomm.hardware.bosch.BNO055IMU
import com.qualcomm.robotcore.util.ElapsedTime
import kotlinx.coroutines.sync.Mutex
import org.firstinspires.ftc.robotcore.external.navigation.Orientation
import org.firstinspires.ftc.teamcode.utils.RobotFuncs
import org.firstinspires.ftc.teamcode.utils.RobotFuncs.KILLALL
import org.firstinspires.ftc.teamcode.utils.RobotFuncs.dashboard
import org.firstinspires.ftc.teamcode.utils.RobotFuncs.logs
import java.lang.Thread.sleep

class Timmy(val name: String) {
    //private val imu: IMU = RobotFuncs.hardwareMap.get(IMU::class.java, name)
    private val imu: BNO055IMU = RobotFuncs.hardwareMap.get(BNO055IMU::class.java, name)
    private val t: Thread
    private var trunning: Boolean = false
    private var initialized: Boolean = false

    private val imuPos = arrayListOf(0.0, 0.0, 0.0)
    private val imuOrientation = arrayListOf(0.0, 0.0, 0.0)

    private fun fixAngOrientation(i: Orientation): Orientation {
        return i
    }

    var yaw: Double = 0.0
        /*get() {
            //while (yawm.tryLock()) {  sleep(1)  }
            //val r = field
            //yawm.unlock()
            //return r
        }*/

    private val yawm = Mutex()

    init {
        val parameters = BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.RADIANS;
        imu.initialize(parameters);
        initialized = true

        /*
        imu.initialize(IMU.Parameters(RevHubOrientationOnRobot(RevHubOrientationOnRobot.LogoFacingDirection.UP, RevHubOrientationOnRobot.UsbFacingDirection.FORWARD)))
        imu.resetYaw()*/

        t = Thread {
            val ep = ElapsedTime()
            ep.reset()
            while (trunning && !KILLALL) {
                //val y = imu.robotYawPitchRollAngles.getYaw(AngleUnit.RADIANS)
                //val y = imu.angularOrientation.firstAngle
                val fixed = fixAngOrientation(imu.angularOrientation)
                val y = fixed.firstAngle.toDouble()
                /*
                while (yawm.tryLock()) {
                    sleep(1)
                }*/
                yaw = y
                //yawm.unlock()
                val tp = TelemetryPacket()
                tp.put("IMU_yaw", y)
                tp.put("IMU_CycleTime", ep.seconds())
                tp.put("IMU_ango", fixed)
                ep.reset()
                dashboard.sendTelemetryPacket(tp)
            }
        }

        logs("Timmy_${name}_Status", "Init")
    }

    fun initThread() {
        if (!trunning) {
            trunning = true
            t.start()
        }
        logs("Timmy_${name}_Status", "InitT")
    }

    fun closeThread() {
        if (trunning) {
            trunning = false
            t.join()
        }
        logs("Timmy_${name}_Status", "CloseT")
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