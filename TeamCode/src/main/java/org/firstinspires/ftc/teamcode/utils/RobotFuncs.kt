package org.firstinspires.ftc.teamcode.utils

import android.graphics.BitmapFactory
import android.graphics.Color
import com.acmerobotics.dashboard.FtcDashboard
import com.acmerobotics.dashboard.config.ValueProvider
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
import org.firstinspires.ftc.teamcode.auto.TrajectorySequence
import org.firstinspires.ftc.teamcode.hardware.CamGirl
import org.firstinspires.ftc.teamcode.hardware.CameraControls.AutoRed
import org.firstinspires.ftc.teamcode.hardware.Controller
import org.firstinspires.ftc.teamcode.hardware.Clown
import org.firstinspires.ftc.teamcode.hardware.Intake
import org.firstinspires.ftc.teamcode.hardware.Intakes
import org.firstinspires.ftc.teamcode.hardware.Intakes.SUpulLuiCostacu
import org.firstinspires.ftc.teamcode.hardware.MServo
import org.firstinspires.ftc.teamcode.hardware.Slides
import org.firstinspires.ftc.teamcode.hardware.Swerve
import org.firstinspires.ftc.teamcode.hardware.Timmy
import org.firstinspires.ftc.teamcode.hardware.ZaPaiplain
import org.firstinspires.ftc.teamcode.pp.Localizer
import org.firstinspires.ftc.teamcode.pp.PP
import org.firstinspires.ftc.teamcode.pp.PurePursuit
import org.firstinspires.ftc.teamcode.pp.ThreeWheelLocalizer
import org.firstinspires.ftc.teamcode.utils.RobotVars.*
import org.firstinspires.ftc.teamcode.utils.Util.angDiff
import org.firstinspires.ftc.teamcode.utils.Util.angNorm
import org.openftc.easyopencv.OpenCvPipeline
import kotlin.math.PI
import kotlin.math.abs
import kotlin.math.atan2
import kotlin.math.cos
import kotlin.math.hypot
import kotlin.math.sin


@Suppress("MemberVisibilityCanBePrivate")
object RobotFuncs {
    lateinit var batteryVoltageSensor: PhotonLynxVoltageSensor
    lateinit var dashboard: FtcDashboard
    lateinit var hardwareMap: HardwareMap
    lateinit var lom: LinearOpMode
    lateinit var telemetry: Telemetry
    lateinit var localizer: Localizer
    lateinit var swerve: Swerve
    lateinit var controller: Controller
    lateinit var pp: PurePursuit
    lateinit var clown: Clown
    lateinit var slides: Slides
    lateinit var intake: Intake
    lateinit var avion: MServo
    lateinit var controlHub: LynxModule
    lateinit var expansionHub: LynxModule
    lateinit var cam: CamGirl
    lateinit var pipeline: OpenCvPipeline
    var KILLALL: Boolean = false

    val etime = ElapsedTime()

    @JvmStatic
    var tp: TelemetryPacket = TelemetryPacket()

    var logmu = Mutex(false)

    @JvmStatic
    fun send_log() {
        while (!logmu.tryLock()) { Thread.sleep(1); }
        dashboard.sendTelemetryPacket(tp)
        tp = TelemetryPacket()
        logmu.unlock()
    }

    @JvmStatic
    fun log(s: String, v: String) {
        if (USE_TELE) {
            while (!logmu.tryLock()) { Thread.sleep(1); }
            tp.put(s, v)
            logmu.unlock()
        }
    }

    @JvmStatic
    fun log(s: String, v: Double) = log(s, String.format("%.4f", v))

    @JvmStatic
    fun log(s: String, v: Any) = log(s, v.toString())

    @JvmStatic
    fun logs(s: String, v: Double) = if (__LOG_STATUS) log(s, String.format("%.4f", v)) else {
    }

    @JvmStatic
    fun logs(s: String, v: Any) = if (__LOG_STATUS) log(s, v.toString()) else {
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
        tp = TelemetryPacket()
        logmu = Mutex(false)
        etime.reset()
        KILLALL = false
    }

    @JvmStatic
    fun shutUp(lopm: LinearOpMode) {
        preinit()
        __SwerveMove = false
        SwerveCanInvertMotor = false
        lom = lopm
        hardwareMap = lom.hardwareMap
        telemetry = lom.telemetry
        batteryVoltageSensor = hardwareMap.getAll(PhotonLynxVoltageSensor::class.java).iterator().next()
        timmy = Timmy("imu")
        clown = Clown("Dif")
        swerve = Swerve()
        controller = Controller()
        localizer = ThreeWheelLocalizer()
        localizer.init(Pose())
        pp = PurePursuit(swerve, localizer)
    }

    val hp = PID(SwerveTurnPIDC)
    var at = ElapsedTime()
    var targetAngle = 0.0
    var angt = ElapsedTime()
    private fun get_angf(fn: Boolean): Double {
        val tx = lom.gamepad1.right_stick_x.toDouble()
        if (fn) {
            if (at.seconds() < SwerveTurnWaitTime) {
                targetAngle = localizer.pose.h
            }
            return if (abs(tx) > 0.05) {
                at.reset()
                hp.reset()
                tx * SwerveManualTurnPower
            } else {
                val er = angDiff(localizer.pose.h, targetAngle)
                if (abs(er) < SwerveTurnMaxDif) {
                    0.0
                } else {
                    hp.update(er)
                }
            }
        } else {
            return tx * SwerveManualTurnPower
        }
    }

    @JvmStatic
    fun moveSwerve(fn: Boolean = false) {
        controller.update()
        if (controller.C1PS == controller.JUST_PRESSED) {
            TimmyCurOff += timmy.yaw
            at.reset()
            log("ResetHeading", TimmyCurOff)
        }
        val speed = hypot(lom.gamepad1.left_stick_x, lom.gamepad1.left_stick_y).toDouble()
        val angle = angNorm(-atan2(lom.gamepad1.left_stick_y, lom.gamepad1.left_stick_x) + Math.PI / 2 - if (USE_FIELD_CENTRIC) timmy.yaw else 0.0)
        //val angle = angNorm(-atan2(lom.gamepad1.left_stick_y, lom.gamepad1.left_stick_x) + Math.PI / 2)
        val correctAngForce = get_angf(fn)
        val fcoef = -(1.0 - lom.gamepad1.right_trigger * 0.6) // No clue why this has to be negative
        swerve.move(speed * fcoef * __FUNNY_SWERVE_COEF, angle, correctAngForce * fcoef)
    }

    @JvmStatic
    fun initma(lopm: LinearOpMode, isauto: Boolean) { /// Init all hardware info
        if (TimmyAddKILLLLLLLL) {
            TimmyCurOff += PI
            TimmyAddKILLLLLLLL = false
        }
        __IsAuto = isauto
        Diffy__UMDC = if (isauto) {
            Diffy__UMDCAuto
        } else {
            Diffy__UMDCOp
        }
        ___KILL_DIFFY_THREADS = false
        angt.reset()
        SwerveCanInvertMotor = false
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
        tp = TelemetryPacket()
        TrajectorySequence().sl(0.5).aa { log("__InitVoltage", batteryVoltageSensor.voltage) }.runAsync()
        send_log()
        if (TimmyToClose) {
            try {
                timmy.close()
            } catch (e: Exception) {
                //log("Timmy wasn't closed", "Idiot")
            }
            timmy = Timmy("imu")
            TimmyToClose = false
        } else {
            try {
                if (!timmy.trunning) {
                    timmy.initThread()
                }
            } catch (e: Exception) {
                log("Timmy wasn't Opened", "Idiot")
                timmy = Timmy("imu")
            }
        }
        controller = Controller()
        slides = Slides()
        localizer = ThreeWheelLocalizer()
        localizer.init(Pose())
        __SwerveMove = USE_SWERVE
        swerve = Swerve()
        swerve.move(0.0, 0.0, 0.0)
        intake = Intake()
        avion = MServo("Pewpew", AvionInchis)
        clown = Clown("Dif")
        if (USE_DIFFY) {
            if (!__IsAuto) {
                TrajectorySequence()
                        .aa { clown.targetPos = DiffyMidUp; clown.targetAngle = DiffyADown }
                        .sl(0.4)
                        .aa { clown.targetPos = DiffyPrepDown; clown.targetAngle = DiffyADown }
                        .runAsyncDiffy()
            } else {
                TrajectorySequence()
                        //.aa { clown.targetPos = DiffyPrepDown; clown.targetAngle = DiffyADown }
                        //.sl(0.3)
                        .aa { clown.ghearaFar?.position = ClownFInchis; clown.ghearaNear?.position = if (__AutoShort) ClownNInchis else ClownNDeschis; }
                        .sl(0.5)
                        .aa { clown.targetPos = DiffyMidUp; clown.targetAngle = DiffyAUp }
                        .runAsyncDiffy()

            }
        }
        pp = PurePursuit(swerve, localizer)
    }

    @JvmStatic
    fun drawRobot() {
        val canvas = tp.fieldOverlay()
        canvas.setStrokeWidth(1)
        canvas.setStroke("#FF00C3")
        canvas.strokeCircle(localizer.pose.x * PP.SCALE, localizer.pose.y * PP.SCALE, PP.robotRadius)
        canvas.setStroke("#00FFC3")
        canvas.strokeLine(localizer.pose.x * PP.SCALE, localizer.pose.y * PP.SCALE,
                (localizer.pose.x * PP.SCALE + PP.robotRadius * cos(localizer.pose.h)), (localizer.pose.y * PP.SCALE + PP.robotRadius * sin(localizer.pose.h)))
    }

    @JvmStatic
    fun initAuto() {
        if (USE_CAMERA) {
            pipeline = ZaPaiplain()
            cam = CamGirl(CameraName, CameraOrientation, 640, 480, pipeline, streaming = true, waitForOpen = true)
        }

        //clown.position = GhearaSINCHIS
        intake.status = Intakes.SDown
    }

    val ep = ElapsedTime()

    @JvmStatic
    fun update() {
        swerve.update()
        slides.update()
        localizer.update()
        intake.update()
        controlHub.clearBulkCache()
        /*
        log("Timmy", timmy.yaw)
        logs("TimmyOF", TimmyCurOff)
        logs("TimmyTime", TimmyLoopTime)*/

        log("0", 0.0)
        tp.put("Framerate", 1 / ep.seconds())
        tp.put("Elapsedtime", etime.seconds())
        ep.reset()
        send_log()
    }

    @JvmStatic
    fun startma() {
        SwerveCanInvertMotor = true
        timmy.initThread()
        localizer.pose = Pose()
        etime.reset()
        at.reset()
    }

    @JvmStatic
    fun endma() { /// Shut down the robot
        KILLALL = true
        batteryVoltageSensor.close()
        if (TimmyToClose) {
            log("SHUT TIMMY OFF!", TimmyToClose)
            send_log()
            timmy.closeThread()
            timmy.close()
            TimmyToClose = false
        }
    }

    @JvmStatic
    fun create_god() {
        try {
            val bmap = BitmapFactory.decodeFile("/storage/self/primary/Hot.png")
            FtcDashboard.getInstance().sendImage(bmap)
        } catch (e: Exception) {
            logst("God could not be created. This is your fault.")
        }
    }
}

class IntiVal<T>(val g: () -> T, val s: (v: T) -> Unit) : ValueProvider<T> {
    override fun get() = g()
    override fun set(v: T) = s(v)
}
