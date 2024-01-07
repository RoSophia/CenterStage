package org.firstinspires.ftc.teamcode.utils

import android.graphics.Color
import com.acmerobotics.dashboard.FtcDashboard
import com.acmerobotics.dashboard.telemetry.TelemetryPacket
import com.outoftheboxrobotics.photoncore.Photon
import com.outoftheboxrobotics.photoncore.PhotonCore
import com.outoftheboxrobotics.photoncore.hardware.PhotonLynxVoltageSensor
import com.qualcomm.hardware.lynx.LynxModule
import com.qualcomm.robotcore.eventloop.opmode.OpMode
import com.qualcomm.robotcore.hardware.HardwareMap
import com.qualcomm.robotcore.hardware.configuration.LynxConstants
import com.qualcomm.robotcore.util.ElapsedTime
import kotlinx.coroutines.runBlocking
import kotlinx.coroutines.sync.Mutex
import org.firstinspires.ftc.robotcore.external.Telemetry
import org.firstinspires.ftc.teamcode.IHRP
import org.firstinspires.ftc.teamcode.hardware.Controller
import org.firstinspires.ftc.teamcode.hardware.Diffy
import org.firstinspires.ftc.teamcode.hardware.MServo
import org.firstinspires.ftc.teamcode.hardware.Motor
import org.firstinspires.ftc.teamcode.hardware.Swerve
import org.firstinspires.ftc.teamcode.hardware.Timmy
import org.firstinspires.ftc.teamcode.pp.Localizer
import org.firstinspires.ftc.teamcode.pp.PurePursuit
import org.firstinspires.ftc.teamcode.pp.ThreeWheelLocalizer
import org.firstinspires.ftc.teamcode.utils.RobotVars.*
import kotlin.math.abs
import kotlin.math.atan2
import kotlin.math.hypot
import kotlin.math.sign


@Suppress("MemberVisibilityCanBePrivate")
object RobotFuncs {
    lateinit var batteryVoltageSensor: PhotonLynxVoltageSensor
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
    lateinit var controlHub: LynxModule
    lateinit var expansionHub: LynxModule
    var KILLALL: Boolean = false

    val etime = ElapsedTime()

    @JvmStatic
    var tp: TelemetryPacket = TelemetryPacket()

    /*@JvmStatic
    val logmu = Mutex()*/

    @JvmStatic
    fun send_log() {
        if (USE_TELE) {
            //runBlocking {
                //logmu.lock()
                dashboard.sendTelemetryPacket(tp)
                //logmu.unlock()
            //}
        }
        tp = TelemetryPacket()
    }

    @JvmStatic
    fun log(s: String, v: String) {
        if (USE_TELE) {
            //val tp = TelemetryPacket()
            //runBlocking {
                //logmu.lock()
                tp.put(s, v)
                //logmu.unlock()
            //}
            //dashboard.sendTelemetryPacket(tp)
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
        logs("Local_Pose", localizer.pose)
        logs("Local_Vel", localizer.poseVel)
        logs("Swerve_Speed", swerve.speed)
        logs("Swerve_Angle", swerve.angle)
        logs("Swerve_TurnPower", swerve.turnPower)
        if (pp.busy) {
            log("PP_LastIndex", pp.lastIndex)
            log("PP_TotalIndexes", pp.ctraj.checkpoints)
        }
        logs("ElapsedTime", etime)
    }

    @JvmStatic
    fun preinit() {
        dashboard = FtcDashboard.getInstance()
        KILLALL = false
    }

    @JvmStatic
    fun shutUp(lopm: OpMode) {
        preinit()
        _MOVE_SWERVE = false
        canInvertMotor = false
        lom = lopm
        hardwareMap = lom.hardwareMap
        telemetry = lom.telemetry
        batteryVoltageSensor = hardwareMap.getAll(PhotonLynxVoltageSensor::class.java).iterator().next()
        timmy = Timmy("imu")
        diffy = Diffy("Dif")
        swerve = Swerve()
        controller = Controller()
        localizer = ThreeWheelLocalizer()
        localizer.init(Pose())
        pp = PurePursuit(swerve, localizer)
    }

    var ale = 0.0
    var at = ElapsedTime() // RB - RF
    var ai = 0.0
    var targetAngle = 0.0
    private fun get_angf(): Double {
        if (USE_DIRECTION_PID) {
            return lom.gamepad1.right_stick_x.toDouble() * IHRP.KILL
        } else {
            if (hypot(lom.gamepad1.right_stick_y, lom.gamepad1.right_stick_x) > 0.5) {
                targetAngle = atan2(lom.gamepad1.right_stick_y, lom.gamepad1.right_stick_x) + Math.PI / 2
            }
            var ae = Util.angDiff(targetAngle, timmy.yaw)
            ae = if (abs(ae) < 0.03) 0.0 else ae
            val ad = (ae - ale) / at.seconds()
            ai += ae * at.seconds()
            at.reset()
            val cf = SwerveHeadPidP * ae + SwerveHeadPidD * ad + SwerveHeadPidI * ai
            val tp = TelemetryPacket()
            tp.put("Target", targetAngle)
            tp.put("TTimmy", timmy.yaw)
            tp.put("TError", ae)
            tp.put("TDeri", ad)
            tp.put("TFors", if (abs(cf) > 0.03) cf + sign(cf) * SwerveHeadPidF else 0.0)
            dashboard.sendTelemetryPacket(tp)
            return if (abs(cf) > 0.03) cf + sign(cf) * SwerveHeadPidF else 0.0
        }
    }


    @JvmStatic
    fun moveSwerve() {
        val speed = hypot(lom.gamepad1.left_stick_x, lom.gamepad1.left_stick_y).toDouble()
        val angle = Util.angNorm(-atan2(lom.gamepad1.left_stick_y, lom.gamepad1.left_stick_x) + Math.PI / 2 + IHRP.OFFSET - timmy.yaw)
        val correctAngForce = get_angf()
        val fcoef = -(1.0 - lom.gamepad1.right_trigger * 0.6) // No clue why this has to be negative
        swerve.move(speed * fcoef, angle, correctAngForce * fcoef)
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
        val lynxModules = hardwareMap.getAll(LynxModule::class.java)
        for (module in lynxModules) {
            module.bulkCachingMode = LynxModule.BulkCachingMode.MANUAL
        }
        if (lynxModules[0].isParent && LynxConstants.isEmbeddedSerialNumber(lynxModules[0].serialNumber)) {
            controlHub = lynxModules[0]
            expansionHub = lynxModules[1]
        } else {
            controlHub = lynxModules[1]
            expansionHub = lynxModules[0]
        }
        controlHub.setConstant(Color.rgb(255, 0, 0))
        expansionHub.setConstant(Color.rgb(255, 100, 20))
        telemetry = lom.telemetry
        //batteryVoltageSensor = hardwareMap.getAll(PhotonLynxVoltageSensor::class.java).iterator().next()
        timmy = Timmy("imu")
        controller = Controller()
        localizer = ThreeWheelLocalizer()
        localizer.init(Pose())
        _MOVE_SWERVE = MOVE_SWERVE
        swerve = Swerve()
        swerve.move(0.0, 0.0, 0.0)
        intake = Motor("Intake", encoder = false, rev = false, overdrive = true)
        ridIntake = MServo("RidIntake", IntakePUp)
        funkyL = MServo("FunkyL", FUNKYLD)
        funkyR = MServo("FunkyR", FUNKYRD)
        clown = MServo("Clown", GhearaSDESCHIS)
        diffy = Diffy("Dif")
        pp = PurePursuit(swerve, localizer)
        logs("RobotFuncs_Status", "FinishInitma")
    }

    val ep = ElapsedTime()
    val timer = ElapsedTime()
    @JvmStatic
    fun update() {
        log("TIME_log", timer.seconds())
        timer.reset()
        controller.update()
        log("TIME_Controller", timer.seconds())
        timer.reset();
        //timmy.update()
        log("TIME_Timmy", timer.seconds())
        timer.reset();
        diffy.update()
        log("TIME_Diffy", timer.seconds())
        timer.reset();
        swerve.update()
        log("TIME_Swerve", timer.seconds())
        timer.reset();
        controlHub.clearBulkCache()
        expansionHub.clearBulkCache()
        log("TIME_Refresh", timer.seconds())
        timer.reset();
        log("Framerate", 1 / ep.seconds())
        ep.reset()
        log_state()
        send_log()
        ep.reset()
    }

    @JvmStatic
    fun startma() { /// Set all values to their starting ones and start the PID threads
        logs("RobotFuncs_Status", "StartStartma")
        canInvertMotor = true
        //pcoef = 12.0 / batteryVoltageSensor.cachedVoltage
        timmy.initThread()
        etime.reset()
        swerve.start()
        //localizer.start()
        diffy.init()
        at.reset()
        logs("RobotFuncs_Status", "FinishStartma")
    }

    @JvmStatic
    fun endma() { /// Shut down the robot
        KILLALL = true
        logs("RobotFuncs_Status", "StartEndma")
        pcoef = 0.0
        //batteryVoltageSensor.close()
        swerve.close()
        timmy.closeThread()
        timmy.close()
        //localizer.close()
        swerve.stop()
        diffy.close()
        logs("RobotFuncs_Status", "FinishEndma")
    }
}
