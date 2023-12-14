package org.firstinspires.ftc.teamcode.utils

import com.acmerobotics.dashboard.FtcDashboard
import com.acmerobotics.dashboard.telemetry.TelemetryPacket
import com.qualcomm.robotcore.eventloop.opmode.OpMode
import com.qualcomm.robotcore.hardware.HardwareMap
import com.qualcomm.robotcore.hardware.VoltageSensor
import com.qualcomm.robotcore.util.ElapsedTime
import org.firstinspires.ftc.robotcore.external.Telemetry
import org.firstinspires.ftc.teamcode.hardware.Controller
import org.firstinspires.ftc.teamcode.hardware.Swerve
import org.firstinspires.ftc.teamcode.hardware.Timmy
import org.firstinspires.ftc.teamcode.pp.Localizer
import org.firstinspires.ftc.teamcode.pp.PurePursuit
import org.firstinspires.ftc.teamcode.pp.ThreeWheelLocalizer
import org.firstinspires.ftc.teamcode.utils.RobotVars.LOG_STATUS
import org.firstinspires.ftc.teamcode.utils.RobotVars.MOVE_SWERVE
import org.firstinspires.ftc.teamcode.utils.RobotVars.USE_TELE
import org.firstinspires.ftc.teamcode.utils.RobotVars.canInvertMotor
import org.firstinspires.ftc.teamcode.utils.RobotVars.pcoef

@Suppress("MemberVisibilityCanBePrivate")
object RobotFuncs {
    lateinit var batteryVoltageSensor: VoltageSensor
    lateinit var dashboard: FtcDashboard
    lateinit var hardwareMap: HardwareMap
    lateinit var lom: OpMode
    lateinit var telemetry: Telemetry
    lateinit var timmy: Timmy
    lateinit var localizer: Localizer
    lateinit var swerve: Swerve
    lateinit var controller: Controller
    lateinit var pp: PurePursuit

    val etime = ElapsedTime()

    @JvmStatic
    fun log(s: String, v: String) {
        if (USE_TELE) {
            val tp = TelemetryPacket()
            tp.put(s, v)
            dashboard.sendTelemetryPacket(tp)
        }
    }

    @JvmStatic
    fun log(s: String, v: Any) = log(s, v.toString())

    @JvmStatic
    fun logs(s: String, v: Any) = if (LOG_STATUS) log(s, v.toString()) else {}

    @JvmStatic
    fun log_state() {
        val pack = TelemetryPacket()

        pack.put("Local_Pose", localizer.pose)
        pack.put("Local_Vel", localizer.poseVel)
        pack.put("Swerve_Speed", swerve.speed)
        pack.put("Swerve_Angle", swerve.angle)
        pack.put("Swerve_TurnPower", swerve.turnPower)
        if (pp.busy) {
            pack.put("PP_LastIndex", pp.lastIndex)
            pack.put("PP_TotalIndexes", pp.ctraj.checkpoints)
        }
        pack.put("ElapsedTime", etime)

        dashboard.sendTelemetryPacket(pack)
    }

    @JvmStatic
    fun preinit() {
        dashboard = FtcDashboard.getInstance()

        /*
        log("ServoModule_LF_MSpeed", "InitVal")
        log("ServoModule_LF_MRever", "InitVal")
        log("CServo_LF_Enc", "InitVal")
        log("CServo_LF_Tar", "InitVal")
        log("CServo_LF_Pow", "InitVal")
        log("ServoModule_LB_MSpeed", "InitVal")
        log("ServoModule_LB_MRever", "InitVal")
        log("CServo_LB_Enc", "InitVal")
        log("CServo_LB_Tar", "InitVal")
        log("CServo_LB_Pow", "InitVal")
        log("ServoModule_RF_MSpeed", "InitVal")
        log("ServoModule_RF_MRever", "InitVal")
        log("CServo_RF_Enc", "InitVal")
        log("CServo_RF_Tar", "InitVal")
        log("CServo_RF_Pow", "InitVal")
        log("ServoModule_RB_MSpeed", "InitVal")
        log("ServoModule_RB_MRever", "InitVal")
        log("CServo_RB_Enc", "InitVal")
        log("CServo_RB_Tar", "InitVal")
        log("CServo_RB_Pow", "InitVal")

        log("Local_Pose", "InitVal")
        log("Local_Vel", "InitVal")

        log("Swerve_Movement", "InitVal")
        log("Swerve_Angle", "InitVal")
        log("Swerve_Speed", "InitVal")
        log("Swerve_TurnPower", "InitVal")

        log("IMU_CycleTime", "InitVal")
        log("IMU_yaw", "InitVal")

        log("ElapsedTime", "InitVal")

        logs("RobotFuncs_Status", "InitVal")
        logs("Timmy_imu_Status", "InitVal")
        logs("Swerve_Status", "InitVal")
        logs("CServo_LB_PID_Status", "InitVal")
        logs("ServoModule_LB_Status", "InitVal")
        logs("CServo_RF_PID_Status", "InitVal")
        logs("ServoModule_RF_Status", "InitVal")
        logs("CServo_RB_PID_Status", "InitVal")
        logs("ServoModule_RB_Status", "InitVal")
        logs("CServo_LF_PID_Status", "InitVal")
        logs("ServoModule_LF_Status", "InitVal")*/



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

    @JvmStatic
    fun shutUp(lopm: OpMode) {
        preinit()
        MOVE_SWERVE = false
        canInvertMotor = false
        lom = lopm
        hardwareMap = lom.hardwareMap
        telemetry = lom.telemetry
        batteryVoltageSensor = hardwareMap.voltageSensor.iterator().next()
        timmy = Timmy("imu")
        swerve = Swerve()
        controller = Controller()
        localizer = ThreeWheelLocalizer()
        pp = PurePursuit(swerve, localizer)
    }

    @JvmStatic
    fun initma(lopm: OpMode) { /// Init all hardware info
        logs("RobotFuncs_Status", "StartInitma")
        canInvertMotor = false
        lom = lopm
        hardwareMap = lom.hardwareMap
        telemetry = lom.telemetry
        batteryVoltageSensor = hardwareMap.voltageSensor.iterator().next()
        timmy = Timmy("imu")
        MOVE_SWERVE = true
        swerve = Swerve()
        swerve.move(0.0, 0.0, 0.0)
        controller = Controller()
        localizer = ThreeWheelLocalizer()
        pp = PurePursuit(swerve, localizer)
        logs("RobotFuncs_Status", "FinishInitma")
    }

    @JvmStatic
    fun startma() { /// Set all values to their starting ones and start the PID threads
        logs("RobotFuncs_Status", "StartStartma")
        canInvertMotor = true
        pcoef = 12.0 / batteryVoltageSensor.voltage
        timmy.initThread()
        etime.reset()
        swerve.start()
        logs("RobotFuncs_Status", "FinishStartma")
    }

    @JvmStatic
    fun endma() { /// Shut down the robot
        logs("RobotFuncs_Status", "StartEndma")
        pcoef = 0.0
        batteryVoltageSensor.close()
        swerve.close()
        timmy.closeThread()
        timmy.close()
        swerve.stop()
        logs("RobotFuncs_Status", "FinishEndma")
    }
}
