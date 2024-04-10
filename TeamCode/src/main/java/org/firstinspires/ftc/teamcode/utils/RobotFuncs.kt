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
import org.firstinspires.ftc.teamcode.auto.AutoFuncs.initQrCamera
import org.firstinspires.ftc.teamcode.hardware.CamGirl
import org.firstinspires.ftc.teamcode.hardware.CameraControls.AutoResult
import org.firstinspires.ftc.teamcode.hardware.Controller
import org.firstinspires.ftc.teamcode.hardware.Clown
import org.firstinspires.ftc.teamcode.hardware.Intake
import org.firstinspires.ftc.teamcode.hardware.Intakes
import org.firstinspires.ftc.teamcode.hardware.MServo
import org.firstinspires.ftc.teamcode.hardware.Slides
import org.firstinspires.ftc.teamcode.hardware.Swerve
import org.firstinspires.ftc.teamcode.hardware.Timmy
import org.firstinspires.ftc.teamcode.hardware.ZaPaiplain
import org.firstinspires.ftc.teamcode.pp.Localizer
import org.firstinspires.ftc.teamcode.pp.PP
import org.firstinspires.ftc.teamcode.pp.PeruWheelLocalizer
import org.firstinspires.ftc.teamcode.pp.PurePursuit
import org.firstinspires.ftc.teamcode.pp.ThreeWheelLocalizer
import org.firstinspires.ftc.teamcode.utils.RobotVars.*
import org.firstinspires.ftc.teamcode.utils.Util.angDiff
import org.firstinspires.ftc.teamcode.utils.Util.angNorm
import org.firstinspires.ftc.vision.VisionPortal
import org.firstinspires.ftc.vision.apriltag.AprilTagPoseFtc
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor
import org.openftc.easyopencv.OpenCvCamera
import org.openftc.easyopencv.OpenCvPipeline
import kotlin.math.PI
import kotlin.math.abs
import kotlin.math.atan2
import kotlin.math.cos
import kotlin.math.hypot
import kotlin.math.sin
import kotlin.random.Random


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
    var cam: CamGirl? = null
    lateinit var pipeline: OpenCvPipeline
    var aprilTag: AprilTagProcessor? = null
    var visionPortal: VisionPortal? = null
    var KILLALL: Boolean = false

    val etime = ElapsedTime()

    @JvmStatic
    var tp: TelemetryPacket = TelemetryPacket()

    var logmu = Mutex(false)

    @JvmStatic
    fun send_log() {
        try {
            synchronized(logmu) {
                dashboard.sendTelemetryPacket(tp)
                tp = TelemetryPacket()
            }
        } catch (e: Exception) {
            try {
                if (logmu.isLocked && logmu.holdsLock(Thread.currentThread())) {
                    logmu.unlock()
                }
            } catch (_: Exception) {}
        }
    }

    @JvmStatic
    fun log(s: String, v: String) {
        if (USE_TELE) {
            try {
                synchronized(logmu) {
                    tp.put(s, v)
                }
            } catch (e: Exception) {
                try {
                    if (logmu.isLocked && logmu.holdsLock(Thread.currentThread())) {
                        logmu.unlock()
                    }
                } catch (_: Exception) {}
            }
        }
    }

    @JvmStatic
    fun log(s: String, v: Double) = log(s, String.format("%.4f", v))

    @JvmStatic
    fun log(s: String, v: Any) = log(s, v.toString())

    @JvmStatic
    fun logs(s: String, v: Double) = if (__LOG_STATUS) log(s, String.format("%.4f", v)) else { }

    @JvmStatic
    fun logs(s: String, v: Any) = if (__LOG_STATUS) log(s, v.toString()) else { }

    @JvmStatic
    fun logst(s: String) {
        if (USE_TELE) {
            synchronized(logmu) {
                tp.addLine(s)
            }
        }
    }

    @JvmStatic
    fun logsst(s: String) = if (__LOG_STATUS) { logst(s) } else {}

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
        localizer = PeruWheelLocalizer()
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
            timmy.resetYaw(0.0)
            at.reset()
            log("ResetHeading", TimmyCurOff)
        }
        val speed = hypot(lom.gamepad1.left_stick_x, lom.gamepad1.left_stick_y).toDouble()
        val angle = angNorm(-atan2(lom.gamepad1.left_stick_y, lom.gamepad1.left_stick_x) + Math.PI / 2 - if (USE_FIELD_CENTRIC) timmy.yaw else 0.0)
        val correctAngForce = get_angf(fn)
        val fcoef = -(1.0 - lom.gamepad1.right_trigger * 0.6) // No clue why this has to be negative
        swerve.move(speed * fcoef * __FUNNY_SWERVE_COEF, angle, correctAngForce * fcoef)
    }

    fun orp(v: AprilTagPoseFtc, h: Double): Pose {
        /// TODO:     AAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAA KLSKMS
        val v2 = (Vec2d(v.y, -v.z) * 2.54).rotated(KMSIMU * h + KMSKMSKMS * PI)
        return Pose(v2.y, v2.x, 0.0)
    }

    @JvmStatic
    fun initma(lopm: LinearOpMode, isauto: Boolean) { /// Init all hardware info
        if (USE_TESTING) {
            USE_CAMERA = false
            AutoResult = Random.nextInt(0, 3)
            log("CurAutoResult", AutoResult)
            send_log()
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
        try {
            if (!timmy.initialized) {
                timmy.init()
            } else {
                if (TimmyToClose) {
                    timmy.close()
                    timmy.init()
                }
            }
            timmy.initThread()
        } catch (e: Exception) {
            //log("Wasn't opened", e.stackTraceToString())
            send_log()
            timmy = Timmy("imu")
            timmy.init()
            timmy.initThread()
        }
        controller = Controller()
        slides = Slides()
        slides.setTarget(RBOT_POS)
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
                        .sl(0.8)
                        .aa { clown.targetPos = DiffyPrepDown; clown.targetAngle = DiffyADown }
                        .runAsyncDiffy()
            } else {
                TrajectorySequence()
                        .aa { clown.ghearaFar?.position = ClownFInchis; clown.ghearaNear?.position = if (__AutoShort) ClownNInchis else ClownNDeschis; }
                        .aa { clown.targetAngle = DiffyAUp; clown.targetPos = DiffyMidUp + 0.1; }
                        .sl(0.4)
                        .aa { clown.targetPos = DiffyMidUp; }
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
        val lp = pp.fixp(localizer.pose)
        canvas.strokeCircle(lp.x * PP.SCALE, lp.y * PP.SCALE, PP.robotRadius)
        canvas.setStroke("#00FFC3")
        canvas.strokeLine(lp.x * PP.SCALE, lp.y * PP.SCALE,
                (lp.x * PP.SCALE + PP.robotRadius * cos(lp.h)), (lp.y * PP.SCALE + PP.robotRadius * sin(lp.h)))
    }

    @JvmStatic
    fun initAuto() {
        if (USE_CAMERA) {
            pipeline = ZaPaiplain()
            cam = CamGirl(CameraName, CameraOrientation, 640, 480, pipeline, streaming = true, waitForOpen = true)
        } else {
            initQrCamera()
        }

        //clown.position = GhearaSINCHIS
        intake.status = Intakes.SDown
    }

    val ep = ElapsedTime()

    @JvmStatic
    fun update() {
        controlHub.clearBulkCache()
        swerve.update()
        slides.update()
        localizer.update()
        intake.update()

        log("0", 0.0)
        //tp.put("Looptime", ep.seconds())
        log("Framerate", 1 / ep.seconds())
        log("Elapsedtime", etime.seconds())
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
        if (!__IsAuto) {
            TimmyCurOff = 0.0
        }
        cam?.camera?.closeCameraDeviceAsync { }
        visionPortal?.close()
        KILLALL = true
        batteryVoltageSensor.close()
        if (TimmyToClose) {
            timmy.closeThread()
            timmy.close()
            TimmyToClose = false
        }
    }

    @JvmStatic
    fun create_god() {
        TrajectorySequence().sl(0.4).aa {
            try {
                val bmap = BitmapFactory.decodeFile("/storage/self/primary/Hot.png")
                FtcDashboard.getInstance().sendImage(bmap)
            } catch (e: Exception) {
                logst("God could not be created. This is your fault.")
            }
        }.runAsync()
    }
}

class IntiVal<T>(val g: () -> T, val s: (v: T) -> Unit) : ValueProvider<T> {
    override fun get() = g()
    override fun set(v: T) = s(v)
}
