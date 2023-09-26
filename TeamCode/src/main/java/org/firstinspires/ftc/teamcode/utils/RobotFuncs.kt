package org.firstinspires.ftc.teamcode.utils

import com.acmerobotics.dashboard.FtcDashboard
import com.acmerobotics.dashboard.telemetry.TelemetryPacket
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import com.qualcomm.robotcore.hardware.HardwareMap
import com.qualcomm.robotcore.hardware.VoltageSensor
import org.firstinspires.ftc.robotcore.external.Telemetry
import org.firstinspires.ftc.teamcode.hardware.Controller
import org.firstinspires.ftc.teamcode.hardware.Motor
import org.firstinspires.ftc.teamcode.hardware.Swerve
import org.firstinspires.ftc.teamcode.hardware.Timmy
import org.firstinspires.ftc.teamcode.utils.RobotVars.pcoef

@Suppress("MemberVisibilityCanBePrivate")
object RobotFuncs {
    lateinit var batteryVoltageSensor: VoltageSensor
    lateinit var dashboard: FtcDashboard
    lateinit var hardwareMap: HardwareMap
    lateinit var lom: LinearOpMode
    lateinit var telemetry: Telemetry
    lateinit var timmy: Timmy
    lateinit var swerve: Swerve
    lateinit var controller: Controller

    @JvmStatic
    fun log_state() {
        val pack = TelemetryPacket()
        dashboard.sendTelemetryPacket(pack)
    }

    @JvmStatic
    fun preinit() {
        // TODO: PhotonCore Currently brokey
        /*
        if (USE_PHOTON) {
            PhotonCore.EXPANSION_HUB.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL)
            PhotonCore.experimental.setMaximumParallelCommands(8)
            PhotonCore.enable()
            PhotonCore.EXPANSION_HUB.clearBulkCache()
        }
         */
    }
    lateinit var lf: Motor

    @JvmStatic
    fun initma(lopm: LinearOpMode) { /// Init all hardware info
        lom = lopm
        hardwareMap = lom.hardwareMap
        telemetry = lom.telemetry
        dashboard = FtcDashboard.getInstance()
        batteryVoltageSensor = hardwareMap.voltageSensor.iterator().next()
        timmy = Timmy("imu")
        swerve = Swerve()
        swerve.move(0.0, 0.0, 0.0)
        controller = Controller()
    }

    @JvmStatic
    fun startma() { /// Set all values to their starting ones and start the PID threads
        pcoef = 12.0 / batteryVoltageSensor.voltage
    }

    @JvmStatic
    fun endma() { /// Shut down the robot
        pcoef = 0.0
        timmy.close()
        batteryVoltageSensor.close()
        swerve.close()
    }
}