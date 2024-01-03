package org.firstinspires.ftc.teamcode.utils

import com.acmerobotics.dashboard.FtcDashboard
import com.acmerobotics.dashboard.telemetry.TelemetryPacket
import com.qualcomm.robotcore.eventloop.opmode.OpMode
import com.qualcomm.robotcore.hardware.HardwareMap
import com.qualcomm.robotcore.hardware.VoltageSensor
import com.qualcomm.robotcore.util.ElapsedTime
import org.firstinspires.ftc.robotcore.external.Telemetry
import org.firstinspires.ftc.teamcode.hardware.Controller
import org.firstinspires.ftc.teamcode.hardware.Diffy
import org.firstinspires.ftc.teamcode.hardware.MServo
import org.firstinspires.ftc.teamcode.hardware.Motor
import org.firstinspires.ftc.teamcode.hardware.Swerve
import org.firstinspires.ftc.teamcode.hardware.Timmy
import org.firstinspires.ftc.teamcode.pp.Localizer
import org.firstinspires.ftc.teamcode.pp.PurePursuit
import org.firstinspires.ftc.teamcode.pp.ThreeWheelLocalizer
import org.firstinspires.ftc.teamcode.utils.RobotVars.FUNKYLD
import org.firstinspires.ftc.teamcode.utils.RobotVars.FUNKYRD
import org.firstinspires.ftc.teamcode.utils.RobotVars.IntakePUp
import org.firstinspires.ftc.teamcode.utils.RobotVars.LOG_STATUS
import org.firstinspires.ftc.teamcode.utils.RobotVars.MOVE_SWERVE
import org.firstinspires.ftc.teamcode.utils.RobotVars.GhearaSDESCHIS
import org.firstinspires.ftc.teamcode.utils.RobotVars.USE_TELE
import org.firstinspires.ftc.teamcode.utils.RobotVars.canInvertMotor
import org.firstinspires.ftc.teamcode.utils.RobotVars.nrRots
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
    lateinit var diffy: Diffy
    lateinit var intake: Motor
    lateinit var ridIntake: MServo
    lateinit var funkyL: MServo
    lateinit var funkyR: MServo
    lateinit var clown: MServo
    var KILLALL: Boolean = false

    val etime = ElapsedTime()

    @JvmStatic
    var tp: TelemetryPacket = TelemetryPacket()

    @JvmStatic
    fun send_log() {
        return
        if (USE_TELE) {
            dashboard.sendTelemetryPacket(tp)
        }
        tp = TelemetryPacket()
    }

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
    fun logs(s: String, v: Any) = if (LOG_STATUS) log(s, v.toString()) else {
    }

    @JvmStatic
    fun logst(s: String) {
        if (USE_TELE) {
            val tp = TelemetryPacket()
            tp.addLine(s)
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
        KILLALL = false
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
        diffy = Diffy("Dif")
        swerve = Swerve()
        controller = Controller()
        localizer = ThreeWheelLocalizer()
        pp = PurePursuit(swerve, localizer)
    }

    @JvmStatic
    fun initma(lopm: OpMode) { /// Init all hardware info
        logs("RobotFuncs_Status", "StartInitma")
        if (nrRots == null) {
            nrRots = HashMap()
        }
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
        intake = Motor("Intake", encoder = false, rev = false, overdrive = true)
        ridIntake = MServo("RidIntake", IntakePUp)
        funkyL = MServo("FunkyL", FUNKYLD)
        funkyR = MServo("FunkyR", FUNKYRD)
        clown = MServo("Clown", GhearaSDESCHIS)
        diffy = Diffy("Dif")
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
        diffy.init()
        logs("RobotFuncs_Status", "FinishStartma")
    }

    @JvmStatic
    fun endma() { /// Shut down the robot
        KILLALL = true
        logs("RobotFuncs_Status", "StartEndma")
        pcoef = 0.0
        batteryVoltageSensor.close()
        swerve.close()
        timmy.closeThread()
        timmy.close()
        swerve.stop()
        diffy.close()
        logs("RobotFuncs_Status", "FinishEndma")
    }
}
