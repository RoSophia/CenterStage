package org.firstinspires.ftc.teamcode.hardware

import androidx.core.graphics.alpha
import com.qualcomm.hardware.rev.RevColorSensorV3
import com.qualcomm.robotcore.hardware.ColorSensor
import com.qualcomm.robotcore.util.ElapsedTime
import org.firstinspires.ftc.teamcode.auto.TrajectorySequence
import org.firstinspires.ftc.teamcode.hardware.Intakes.SUpulLuiCostacu
import org.firstinspires.ftc.teamcode.utils.PeriodicRunner
import org.firstinspires.ftc.teamcode.utils.RobotFuncs
import org.firstinspires.ftc.teamcode.utils.RobotFuncs.controller
import org.firstinspires.ftc.teamcode.utils.RobotFuncs.etime
import org.firstinspires.ftc.teamcode.utils.RobotFuncs.expansionHub
import org.firstinspires.ftc.teamcode.utils.RobotFuncs.hardwareMap
import org.firstinspires.ftc.teamcode.utils.RobotFuncs.intake
import org.firstinspires.ftc.teamcode.utils.RobotFuncs.log
import org.firstinspires.ftc.teamcode.utils.RobotFuncs.logs
import org.firstinspires.ftc.teamcode.utils.RobotFuncs.lom
import org.firstinspires.ftc.teamcode.utils.RobotVars.*
import java.util.Vector
import kotlin.Exception

class Clown(name: String) {
    private var USE_GHEARA_NEAR = true
    private var USE_GHEARA_FAR = true
    private var USE_GELENK = true

    private val threads = Vector<Thread>()
    private val RS = MServo(name + "RS", false, if (USE_DIFFY) if (__IsAuto) DiffyDown / 2.0 + DiffyADown else DiffyPrepDown / 2.0 + DiffyADown else null)
    private val LS = MServo(name + "LS", true, if (USE_DIFFY) if (__IsAuto) DiffyDown / 2.0 - DiffyADown else DiffyPrepDown / 2.0 - DiffyADown else null)
    private val RSE = AbsEnc(name + "RSE", DiffyEncROff)
    private val LSE = AbsEnc(name + "LSE", DiffyEncLOff, true)
    val ghearaNear = if (USE_GHEARA_NEAR) MServo("GhearaNear", true, if (USE_DIFFY) if (__IsAuto) ClownNDeschis else ClownNDeschis else null) else null
    val ghearaFar = if (USE_GHEARA_FAR) MServo("GhearaFar", false, if (USE_DIFFY) if (__IsAuto) ClownNInchis else ClownFDeschis else null) else null
    private val gelenk = if (USE_GELENK) MServo("Gelenk", false, if (USE_DIFFY) GelenkCenter else null) else null

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

    fun sensorReadout() = (if (sensorNearDist > SensorsMinDist) 2 else 0) or
            (if (sensorFarDist > SensorsMinDist) 1 else 0)

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
        }
    }

    /// <+-2 = up turns
    /// -100 = down
    /// -101 = up preload
    /// -102 = pixel secured
    private var curState = -100

    private var sexPixelTraj = TrajectorySequence()
    private var goUpTraj = TrajectorySequence()
    private var goUp2Traj = TrajectorySequence()
    private var goPreloadUpTraj = TrajectorySequence()
    private var goDownTraj = TrajectorySequence()
    private var goPreloadDownTraj = TrajectorySequence()

    init {
        run {
            goUpTraj.aa {
                targetPos = DiffyDown
                targetAngle = DiffyADown
                gelenk?.position = GelenkCenter
            }
                    .sl(ClownWait1)
                    .aa {
                        close()
                    }
                    .sl(ClownWait2)
                    .aa {
                        intake.status = SUpulLuiCostacu
                        //__UPDATE_DIFFY = true
                    }
                    .sl(ClownWait3)
                    .aa {
                        curState = nextA
                        targetPos = DiffyUp
                    }
                    .sl(ClownWait4)
                    //goUpTraj.wt { lastDiffyPos > DiffyWaitUpTurn }
                    .aa {
                        //__UPDATE_DIFFY = false
                        gelenk?.position = GelenkCenter + curState * GelenkDif
                        targetAngle = DiffyAUp
                        targetPos = DiffyUp
                    }
        } /// GoUp

        run {
            sexPixelTraj.aa {
                targetPos = DiffyDown
                targetAngle = DiffyADown
                gelenk?.position = GelenkCenter
            }
                    .sl(ClownWait1)
                    .aa {
                        close()
                    }
                    .sl(ClownWait2)
                    .aa {
                        curState = -102
                        targetPos = DiffyPrepDown
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
            goDownTraj.aa { /// TODO: Check if already open
                open()
            }
                    .sl(ClownWaitDown1)
                    .aa {
                        curState = -100
                        targetPos = DiffyPrepDown
                        gelenk?.position = GelenkCenter
                        //__UPDATE_DIFFY = true
                    }
                    .sl(ClownWaitDown2)
                    //goDownTraj.wt { lastDiffyPos < DiffyWaitDownTurn }
                    .aa {
                        //__UPDATE_DIFFY = false
                        targetPos = DiffyPrepDown
                        targetAngle = DiffyADown
                        gelenk?.position = GelenkCenter
                    }
        } /// Go Down

        run {
            goPreloadUpTraj.aa {
                intake.status = SUpulLuiCostacu
            }
                    .sl(ClownPWait2)
                    .aa { targetPos = DiffyPreloadUp }
                    .sl(ClownPWait3)
                    .aa {
                        targetAngle = DiffyAUp
                    }
        } /// GoPreloadUp

        run {
            goPreloadDownTraj.aa {
                curState = -100
                targetPos = DiffyPrepDown
                gelenk?.position = GelenkCenter
                ghearaFar?.position = ClownFDeschis
                //__UPDATE_DIFFY = true
            }
                    .sl(ClownWaitDown2)
                    //goPreloadDownTraj.wt { lastDiffyPos < DiffyWaitDownTurn }
                    .aa {
                        targetAngle = DiffyADown
                        //__UPDATE_DIFFY = false
                    }
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
            goPreloadUpTraj.runAsyncDiffy()
        }
    }

    fun goPreloadDown() {
        if (USE_DIFFY) {
            goPreloadDownTraj.runAsyncDiffy()
        }
    }

    fun goLeft() {
        if (USE_DIFFY) {
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

    fun goRight() {
        if (USE_DIFFY) {
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
            killextrathreads()
            sexPixelTraj.runAsyncDiffy()
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
                }

                -101, -102 -> {
                    threads.add(goUp2Traj.runAsyncDiffy())
                }

                else -> {
                    curState = a
                    updateAngle()
                }
            }
        }
    }

    fun goDown() {
        if (USE_DIFFY) {
            killextrathreads()
            threads.add(goDownTraj.runAsyncDiffy())
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