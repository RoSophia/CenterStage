package org.firstinspires.ftc.teamcode.utils

import com.acmerobotics.dashboard.FtcDashboard
import com.acmerobotics.dashboard.telemetry.TelemetryPacket
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
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

        log("RobotFuncs_Status", "InitVal")
        log("Timmy_imu_Status", "InitVal")

        log("ElapsedTime", "InitVal")

        log("Swerve_Status", "InitVal")
        log("CServo_LB_PID_Status", "InitVal")
        log("ServoModule_LB_Status", "InitVal")
        log("CServo_RF_PID_Status", "InitVal")
        log("ServoModule_RF_Status", "InitVal")
        log("CServo_RB_PID_Status", "InitVal")
        log("ServoModule_RB_Status", "InitVal")
        log("CServo_LF_PID_Status", "InitVal")
        log("ServoModule_LF_Status", "InitVal")



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
    fun initma(lopm: OpMode) { /// Init all hardware info
        log("RobotFuncs_Status", "StartInitma")
        canInvertMotor = false
        lom = lopm
        hardwareMap = lom.hardwareMap
        telemetry = lom.telemetry
        batteryVoltageSensor = hardwareMap.voltageSensor.iterator().next()
        timmy = Timmy("imu")
        swerve = Swerve()
        swerve.move(0.0, 0.0, 0.0)
        controller = Controller()
        localizer = ThreeWheelLocalizer()
        pp = PurePursuit(swerve, localizer)
        log("RobotFuncs_Status", "FinishInitma")
    }

    @JvmStatic
    fun startma() { /// Set all values to their starting ones and start the PID threads
        log("RobotFuncs_Status", "StartStartma")
        canInvertMotor = true
        pcoef = 12.0 / batteryVoltageSensor.voltage
        timmy.initThread()
        etime.reset()
        log("RobotFuncs_Status", "FinishStartma")
    }

    @JvmStatic
    fun endma() { /// Shut down the robot
        log("RobotFuncs_Status", "StartEndma")
        pcoef = 0.0
        batteryVoltageSensor.close()
        swerve.close()
        timmy.closeThread()
        timmy.close()
        log("RobotFuncs_Status", "FinishEndma")
    }
}
