package org.firstinspires.ftc.teamcode.hardware

import com.acmerobotics.dashboard.telemetry.TelemetryPacket
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot
import com.qualcomm.robotcore.hardware.IMU
import com.qualcomm.robotcore.util.ElapsedTime
import kotlinx.coroutines.sync.Mutex
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit
import org.firstinspires.ftc.teamcode.utils.RobotFuncs
import org.firstinspires.ftc.teamcode.utils.RobotFuncs.dashboard
import org.firstinspires.ftc.teamcode.utils.RobotFuncs.log
import org.firstinspires.ftc.teamcode.utils.RobotFuncs.logs
import java.lang.Thread.sleep

class Timmy(val name: String) {
    private val imu: IMU = RobotFuncs.hardwareMap.get(IMU::class.java, name)
    private val t: Thread
    private var trunning: Boolean = false
    private var initialized: Boolean = false

    var yaw: Double = 0.0
        get() {
            while (yawm.tryLock()) {
                sleep(1)
            }
            val r = field
            yawm.unlock()
            return r
        }

    private val yawm = Mutex()

    init {
        imu.initialize(IMU.Parameters(RevHubOrientationOnRobot(RevHubOrientationOnRobot.LogoFacingDirection.UP, RevHubOrientationOnRobot.UsbFacingDirection.FORWARD)))
        initialized = true
        imu.resetYaw()

        t = Thread {
            val ep = ElapsedTime()
            ep.reset()
            while (trunning) {
                val y = imu.robotYawPitchRollAngles.getYaw(AngleUnit.RADIANS)
                /*
                while (yawm.tryLock()) {
                    sleep(1)
                }
                yaw = y
                yawm.unlock()
                 */
                val tp = TelemetryPacket()
                tp.put("IMU_yaw", y)
                tp.put("IMU_CycleTime", ep.seconds())
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