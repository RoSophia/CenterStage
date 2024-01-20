package org.firstinspires.ftc.teamcode.utils

import android.graphics.Color
import android.transition.Slide
import com.acmerobotics.dashboard.FtcDashboard
import com.acmerobotics.dashboard.telemetry.TelemetryPacket
import com.outoftheboxrobotics.photoncore.hardware.PhotonLynxVoltageSensor
import com.qualcomm.hardware.lynx.LynxModule
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import com.qualcomm.robotcore.hardware.HardwareMap
import com.qualcomm.robotcore.hardware.configuration.LynxConstants
import com.qualcomm.robotcore.util.ElapsedTime
import kotlinx.coroutines.runBlocking
import kotlinx.coroutines.sync.Mutex
import org.firstinspires.ftc.robotcore.external.Telemetry
import org.firstinspires.ftc.teamcode.hardware.CamGirl
import org.firstinspires.ftc.teamcode.hardware.Controller
import org.firstinspires.ftc.teamcode.hardware.Diffy
import org.firstinspires.ftc.teamcode.hardware.Intake
import org.firstinspires.ftc.teamcode.hardware.MServo
import org.firstinspires.ftc.teamcode.hardware.Slides
import org.firstinspires.ftc.teamcode.hardware.Swerve
import org.firstinspires.ftc.teamcode.hardware.Timmy
import org.firstinspires.ftc.teamcode.hardware.ZaPaiplain
import org.firstinspires.ftc.teamcode.pp.Localizer
import org.firstinspires.ftc.teamcode.pp.PurePursuit
import org.firstinspires.ftc.teamcode.pp.ThreeWheelLocalizer
import org.firstinspires.ftc.teamcode.utils.RobotVars.*
import org.firstinspires.ftc.teamcode.utils.Util.angNorm
import org.openftc.easyopencv.OpenCvPipeline
import kotlin.math.abs
import kotlin.math.atan2
import kotlin.math.hypot
import kotlin.math.sign


@Suppress("MemberVisibilityCanBePrivate")
object RobotFuncs {
    lateinit var batteryVoltageSensor: PhotonLynxVoltageSensor
    lateinit var dashboard: FtcDashboard
    lateinit var hardwareMap: HardwareMap
    lateinit var lom: LinearOpMode
    lateinit var telemetry: Telemetry
    lateinit var timmy: Timmy
    lateinit var localizer: Localizer
    lateinit var swerve: Swerve
    lateinit var controller: Controller
    lateinit var pp: PurePursuit
    lateinit var diffy: Diffy
    lateinit var slides: Slides
    lateinit var intake: Intake
    lateinit var funkyL: MServo
    lateinit var funkyR: MServo
    lateinit var clown: MServo
    lateinit var controlHub: LynxModule
    lateinit var expansionHub: LynxModule
    lateinit var cam: CamGirl
    lateinit var pipeline: OpenCvPipeline
    var KILLALL: Boolean = false

    val etime = ElapsedTime()

    @JvmStatic
    var tp: TelemetryPacket = TelemetryPacket()

    val logmu = Mutex()

    @JvmStatic
    fun send_log() {
        runBlocking {
            logmu.lock()
            dashboard.sendTelemetryPacket(tp)
            logmu.unlock()
        }
        tp = TelemetryPacket()
    }

    @JvmStatic
    fun log(s: String, v: String) {
        if (USE_TELE) {
            runBlocking {
                logmu.lock()
                tp.put(s, v)
                logmu.unlock()
            }
        }
    }

    @JvmStatic
    fun log(s: String, v: Double) = log(s, String.format("%.4f", v))
    @JvmStatic
    fun log(s: String, v: Any) = log(s, v.toString())

    @JvmStatic
    fun logs(s: String, v: Double) = if (LOG_STATUS) log(s, String.format("%.4f", v)) else {
    }

    @JvmStatic
    fun logs(s: String, v: Any) = if (LOG_STATUS) log(s, v.toString()) else {
    }

    @JvmStatic
    fun logst(s: String) {
        if (USE_TELE) {
            runBlocking {
                logmu.lock()
                tp.addLine(s)
                logmu.unlock()
            }
        }
    }

    @JvmStatic
    fun preinit() {
        dashboard = FtcDashboard.getInstance()
        KILLALL = false
    }

    @JvmStatic
    fun shutUp(lopm: LinearOpMode) {
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
        localizer.init(LocalizerInitPos)
        pp = PurePursuit(swerve, localizer)
    }

    var ale = 0.0
    var at = ElapsedTime()
    var ai = 0.0
    var targetAngle = 0.0
    var angt = ElapsedTime()
    private fun get_angf(): Double {
        if (USE_DIRECTION_PID) {
            if (abs(lom.gamepad1.right_stick_x.toDouble()) > 0.02) {
                angt.reset()
                return lom.gamepad1.right_stick_x.toDouble()
            } else {
                if (angt.seconds() < SwerveMaxKeepAngTime) {
                    targetAngle = timmy.yaw
                    ai = 0.0
                }
                var ae = Util.angDiff(targetAngle, timmy.yaw)
                ae = if (abs(ae) < 0.03) 0.0 else ae
                val ad = (ae - ale) / at.seconds()
                ai += ae * at.seconds()
                at.reset()
                val cf = SwerveHeadPidP * ae + SwerveHeadPidD * ad + SwerveHeadPidI * ai
                log("Target", targetAngle)
                log("TTimmy", timmy.yaw)
                log("TError", ae)
                log("TDeri", ad)
                log("TFors", if (abs(cf) > 0.03) cf + sign(cf) * SwerveHeadPidF else 0.0)
                return if (abs(cf) > 0.03) cf + sign(cf) * SwerveHeadPidF else 0.0
            }
        } else {
            return lom.gamepad1.right_stick_x.toDouble()
        }
    }

    @JvmStatic
    fun moveSwerve() {
        controller.update()
        val speed = hypot(lom.gamepad1.left_stick_x, lom.gamepad1.left_stick_y).toDouble()
        val angle = angNorm(-atan2(lom.gamepad1.left_stick_y, lom.gamepad1.left_stick_x) + Math.PI / 2 - if (USE_FIELD_CENTRIC) timmy.yaw else 0.0)
        //val angle = angNorm(-atan2(lom.gamepad1.left_stick_y, lom.gamepad1.left_stick_x) + Math.PI / 2)
        val correctAngForce = get_angf()
        val fcoef = -(1.0 - lom.gamepad1.right_trigger * 0.6) // No clue why this has to be negative
        swerve.move(speed * fcoef, angle, correctAngForce * fcoef)
    }

    @JvmStatic
    fun initma(lopm: LinearOpMode) { /// Init all hardware info
        angt.reset()
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
        batteryVoltageSensor = hardwareMap.getAll(PhotonLynxVoltageSensor::class.java).iterator().next()
        timmy = Timmy("imu")
        controller = Controller()
        slides = Slides()
        localizer = ThreeWheelLocalizer()
        localizer.init(LocalizerInitPos)
        _MOVE_SWERVE = MOVE_SWERVE
        swerve = Swerve()
        swerve.move(0.0, 0.0, 0.0)
        intake = Intake()
        funkyL = MServo("FunkyL")
        funkyR = MServo("FunkyR")
        clown = MServo("Clown", GhearaSDESCHIS)
        diffy = Diffy("Dif")
        pp = PurePursuit(swerve, localizer)
    }

    @JvmStatic
    fun initAuto() {
        pipeline = ZaPaiplain(640, 480)
        cam = CamGirl(CameraName, CameraOrientation, 640, 480, pipeline, streaming = true, waitForOpen = true)
    }

    val ep = ElapsedTime()

    @JvmStatic
    fun update() {
        swerve.update()
        intake.update()
        slides.update()
        localizer.update()
        controlHub.clearBulkCache()
        //expansionHub.clearBulkCache()
        log("Timmy", timmy.yaw)
        log("TimmyTime", TIMMYA)
        log("0", 0.0)
        tp.put("Framerate", 1 / ep.seconds())
        ep.reset()
        send_log()
    }

    @JvmStatic
    fun startma() { /// Set all values to their starting ones and start the PID threads
        canInvertMotor = true
        //pcoef = 13.0 / batteryVoltageSensor.cachedVoltage
        timmy.initThread()
        etime.reset()
        swerve.start()
        //localizer.start()
        at.reset()
    }

    @JvmStatic
    fun endma() { /// Shut down the robot
        KILLALL = true
        //pcoef = 0.0
        batteryVoltageSensor.close()
        swerve.close()
        timmy.closeThread()
        timmy.close()
        //localizer.close()
        swerve.stop()
    }
}
