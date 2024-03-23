package org.firstinspires.ftc.teamcode.hardware

import androidx.core.graphics.alpha
import com.qualcomm.robotcore.hardware.ColorSensor
import org.firstinspires.ftc.teamcode.hardware.CameraControls.AutoRed
import org.firstinspires.ftc.teamcode.hardware.Intakes.SUpulLuiCostacu
import org.firstinspires.ftc.teamcode.utils.PeriodicRunner
import org.firstinspires.ftc.teamcode.utils.RobotFuncs.etime
import org.firstinspires.ftc.teamcode.utils.RobotFuncs.expansionHub
import org.firstinspires.ftc.teamcode.utils.RobotFuncs.hardwareMap
import org.firstinspires.ftc.teamcode.utils.RobotFuncs.intake
import org.firstinspires.ftc.teamcode.utils.RobotFuncs.log
import org.firstinspires.ftc.teamcode.utils.RobotFuncs.logs
import org.firstinspires.ftc.teamcode.utils.RobotFuncs.lom
import org.firstinspires.ftc.teamcode.utils.RobotFuncs.slides
import org.firstinspires.ftc.teamcode.utils.RobotVars.*
import org.firstinspires.ftc.teamcode.utils.TrajectorySequence
import org.firstinspires.ftc.teamcode.utils.Util.epsEq
import java.util.Vector
import kotlin.Exception

class Clown(name: String) {
    private var USE_GHEARA_NEAR = true
    private var USE_GHEARA_FAR = true
    private var USE_GELENK = true

    private val threads = Vector<Thread>()
    private val RS = MServo(name + "RS", false, if (USE_DIFFY) if (__IsAuto) DiffyMidUp / 2.0 + DiffyAUp else DiffyPrepDown / 2.0 + DiffyADown else null)
    private val LS = MServo(name + "LS", true, if (USE_DIFFY) if (__IsAuto) DiffyMidUp / 2.0 - DiffyAUp else DiffyPrepDown / 2.0 - DiffyADown else null)
    private val RSE = AbsEnc(name + "RSE", DiffyEncROff)
    private val LSE = AbsEnc(name + "LSE", DiffyEncLOff, true)
    val ghearaNear = if (USE_GHEARA_NEAR) MServo("GhearaNear", true, if (USE_DIFFY) if (__IsAuto && AutoRed) ClownNDeschis else ClownNDeschis else null) else null
    val ghearaFar = if (USE_GHEARA_FAR) MServo("GhearaFar", false, if (USE_DIFFY) if (__IsAuto && AutoRed) ClownNDeschis else ClownFDeschis else null) else null
    val gelenk = if (USE_GELENK) MServo("Gelenk", false, if (USE_DIFFY) GelenkCenter else null) else null

    private val sensorNear = if (USE_SENSORS) hardwareMap.get(ColorSensor::class.java, "SensorNear") else null
    private val sensorFar = if (USE_SENSORS) hardwareMap.get(ColorSensor::class.java, "SensorFar") else null
    private var sensorNearDist = 0
    private var sensorFarDist = 0

    init {
        __UPDATE_SENSORS = false
    }

    private var lastDiffyPos = 0.0

    val actualDiffyPos: Double
        get() {
            expansionHub.clearBulkCache()
            var rs = RSE.kmsang
            log("RSAng", rs)
            if (rs > 5.0) {
                rs = 0.0
            }
            var ls = LSE.kmsang
            log("LSAng", ls)
            if (ls > 5.0) {
                ls = 0.0
            }
            val curP = (rs + ls) / 2.0
            log("LSRes", curP)
            log("ACcessing diffy aaaaaaaaaaaaaa", etime.seconds())
            return curP
        }

    val diffyPosThread = PeriodicRunner({ __UPDATE_DIFFY }, { actualDiffyPos }, { v: Double -> lastDiffyPos = v }, 0.0, 10.0, "Diffy")

    val sensorNearThread = PeriodicRunner({ __UPDATE_SENSORS }, {
        sensorNear?.argb()?.alpha ?: 0
    }, { v: Int -> sensorNearDist = v }, 0, 10.0, "Near")
    val sensorFarThread = PeriodicRunner({ __UPDATE_SENSORS }, {
        sensorFar?.argb()?.alpha ?: 0
    }, { v: Int -> sensorFarDist = v }, 0, 10.0, "Far")


    fun sensorReadout(): Int {
        //log("SensorReadout", res)
        return (if (sensorNearDist > SensorsMinDist) 2 else 0) or
                (if (sensorFarDist > SensorsMinDist) 1 else 0)
    }

    private fun updateTarget() {
        if (USE_DIFFY) {
            RS.position = (targetPos / 2.0 + targetAngle) + DiffyROff
            LS.position = (targetPos / 2.0 - targetAngle) + DiffyLOff
        }
    }

    private fun updateAngle() {
        if (USE_DIFFY) {
            gelenk?.position = GelenkCenter + curState * GelenkDif
            targetAngle = DiffyAUp
            targetPos = DiffyUp
        }
    }

    /// <+-2 = up turns
    /// -100 = down
    /// -101 = up preload
    /// -102 = pixel secured
    var curState = -100

    private var sexPixelTraj = TrajectorySequence()
    private var goUpTraj = TrajectorySequence()
    private var goUp2Traj = TrajectorySequence()
    private var goPreloadUpTraj = TrajectorySequence()
    private var goDownTraj = TrajectorySequence()
    private var goPreloadDownTraj = TrajectorySequence()
    private var ampUp = AsymmetricMotionProfile(Diffy__UMVEL, Diffy__UMAC, Diffy__UMDC)
    private var ampDown = AsymmetricMotionProfile(Diffy__MVEL, Diffy__MAC, Diffy__MDC)

    init {
        run {
            goUpTraj.aa {
                targetPos = DiffyDown
                targetAngle = DiffyADown
                gelenk?.position = GelenkCenter
            }
                    .sl(ClownWait1)
                    .aa { close(); intake.status = SUpulLuiCostacu }
                    .sl(ClownWait2)
                    .aa { ampUp.setMotion(DiffyPrepDown, DiffyUp, 0.0); }
                    .wt {
                        ampUp.update(); targetPos = ampUp.position
                        if (targetPos > DiffyWaitUpTurn) {
                            targetAngle = DiffyAUp; gelenk?.position = GelenkCenter + curState * GelenkDif; curState = nextA
                        }
                        epsEq(ampUp.position, ampUp.finalPosition)
                    }
                    .aa { targetAngle = DiffyAUp; targetPos = DiffyUp }
        } /// GoUp

        run {
            sexPixelTraj.aa {
                targetPos = DiffyDown
                targetAngle = DiffyADown
                gelenk?.position = GelenkCenter
            }
                    .sl(ClownWait1)
                    .aa { close() }
                    .sl(ClownWait2)
                    .aa {
                        curState = -102
                        targetPos = DiffyHoldDown
                    }
        } /// Sex

        run {
            goUp2Traj.aa {
                intake.status = SUpulLuiCostacu
                curState = nextA
                targetAngle = DiffyADown
                targetPos = DiffyUp
                //__UPDATE_DIFFY = true
            }
                    .sl(ClownWait4)
                    //goUp2Traj.wt { lastDiffyPos > DiffyWaitUpTurn }
                    .aa {
                        //__UPDATE_DIFFY = false
                        gelenk?.position = GelenkCenter + curState * GelenkDif
                        targetAngle = DiffyAUp
                        targetPos = DiffyUp
                    }
        } /// Go Up Already Taken

        run {
            goDownTraj.aa { open() }
                    .sl(ClownWaitDown1)
                    .aa { slides.setTarget(RBOT_POS); ampDown.setMotion(DiffyUp, DiffyPrepDown, 0.0); }
                    .wt {
                        ampDown.update(); targetPos = ampDown.position
                        if (targetPos < DiffyWaitDownTurn) {
                            targetAngle = DiffyADown; gelenk?.position = GelenkCenter; curState = -100
                        }
                        epsEq(ampDown.position, ampDown.finalPosition)
                    }
                    .sl(ClownWaitDown2)
                    .aa { targetPos = DiffyPrepDown; gelenk?.position = GelenkCenter }
        } /// Go Down

        run {
            goPreloadUpTraj.aa { targetPos = DiffyPreloadUp }
        } /// GoPreloadUp

        run {
            goPreloadDownTraj.aa {
                curState = -100
                targetPos = DiffyMidDown
                gelenk?.position = GelenkCenter
                ghearaFar?.position = ClownFDeschis
            }
                    .sl(ClownWaitDown4)
                    .aa { targetAngle = DiffyADown }
                    .sl(ClownWaitDown5)
                    .aa { targetPos = DiffyPrepDown }
        } /// GoPreloadDown
    }

    private fun killextrathreads() {
        try {
            ___KILL_DIFFY_THREADS = true
            __UPDATE_DIFFY = false
            for (t in threads) {
                if (t.isAlive) {
                    t.join()
                }
            }
            threads.clear()
        } catch (e: Exception) {
            log("got this exception amte", e.stackTraceToString())
        }
        ___KILL_DIFFY_THREADS = false
    }

    fun goPreloadUp() {
        if (USE_DIFFY) {
            threads.add(goPreloadUpTraj.runAsyncDiffy())
            //logs("Create traj ${threads.lastElement().id}", "PreloadUp")
        }
    }

    fun goPreloadDown() {
        if (USE_DIFFY) {
            threads.add(goPreloadDownTraj.runAsyncDiffy())
            //logs("Create traj ${threads.lastElement().id}", "PreloadDown")
        }
    }

    fun goLeft() {
        if (USE_DIFFY) {
            if (curState > -100) {
                killextrathreads()
                if (curState == -100) {
                    goUp(-1)
                } else {
                    if (curState > -2) {
                        --curState
                        updateAngle()
                    }
                }
            }
        }
    }

    fun goRight() {
        if (USE_DIFFY) {
            if (curState > -100) {
                killextrathreads()
                if (curState == -100) {
                    goUp(1)
                } else {
                    if (curState < 2) {
                        ++curState
                        updateAngle()
                    }
                }
            }
        }
    }

    fun close() {
        if (USE_DIFFY) {
            ghearaNear?.position = ClownNInchis
            ghearaFar?.position = ClownFInchis
        }
    }

    fun open() {
        if (USE_DIFFY) {
            ghearaNear?.position = ClownNDeschis
            ghearaFar?.position = ClownFDeschis
        }
    }

    var nextA = 0

    fun catchPixel() {
        if (USE_DIFFY) {
            if (curState == -100 || curState == -102) {
                killextrathreads()
                threads.add(sexPixelTraj.runAsyncDiffy())
                //logs("Create traj ${threads.lastElement().id}", "SexPixel")
            }
        }
    }

    fun goUp(a: Int) {
        if (USE_DIFFY) {
            lom.gamepad2.stopRumble()
            killextrathreads()
            nextA = a
            when (curState) {
                -100 -> {
                    threads.add(goUpTraj.runAsyncDiffy())
                    //logs("Create traj ${threads.lastElement().id}", "GOUP")
                }

                -101, -102 -> {
                    threads.add(goUp2Traj.runAsyncDiffy())
                    //logs("Create traj ${threads.lastElement().id}", "GOUP2")
                }

                else -> {
                    curState = a
                    updateAngle()
                }
            }
            if (threads.size == 2) {
                log("CURHTREADSSS", threads.size)
            }
        }
    }

    fun goDown() {
        if (USE_DIFFY) {
            killextrathreads()
            if (curState != -102 && curState != -100) {
                threads.add(goDownTraj.runAsyncDiffy())
                //logs("Create traj ${threads.lastElement().id}", "GODOWN")
            } else {
                gelenk?.position = GelenkCenter
                targetAngle = DiffyADown
                targetPos = DiffyPrepDown
                curState = -100
                open()
            }
            if (threads.size == 2) {
                log("CURHTREADSSSDOWN", threads.size)
            }
        }
    }

    var targetAngle = 0.0
        set(v) {
            if (v != field) {
                field = v
                updateTarget()
            }
        }

    var targetPos = 0.0
        set(v) {
            if (v != field) {
                field = v
                updateTarget()
            }
        }
}