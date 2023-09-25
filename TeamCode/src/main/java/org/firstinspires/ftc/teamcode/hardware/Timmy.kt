package org.firstinspires.ftc.teamcode.hardware

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot
import com.qualcomm.robotcore.hardware.IMU
import kotlinx.coroutines.sync.Mutex
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit
import org.firstinspires.ftc.teamcode.utils.RobotFuncs
import java.lang.Thread.sleep

class Timmy(name: String) {
    private val imu: IMU
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
        imu = RobotFuncs.hardwareMap.get(IMU::class.java, name)
        imu.initialize(IMU.Parameters(RevHubOrientationOnRobot(RevHubOrientationOnRobot.LogoFacingDirection.LEFT, RevHubOrientationOnRobot.UsbFacingDirection.BACKWARD)))
        initialized = true
        imu.resetYaw()

        t = Thread {
            while (trunning) {
                val y = imu.robotYawPitchRollAngles.getYaw(AngleUnit.RADIANS)
                while (yawm.tryLock()) {
                    sleep(1)
                }
                yaw = y
                yawm.unlock()
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