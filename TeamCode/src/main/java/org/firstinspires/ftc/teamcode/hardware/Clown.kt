package org.firstinspires.ftc.teamcode.hardware

import com.qualcomm.robotcore.util.ElapsedTime
import org.firstinspires.ftc.teamcode.auto.TrajectorySequence
import org.firstinspires.ftc.teamcode.hardware.Intakes.SUpulLuiCostacu
import org.firstinspires.ftc.teamcode.utils.RobotFuncs.etime
import org.firstinspires.ftc.teamcode.utils.RobotFuncs.expansionHub
import org.firstinspires.ftc.teamcode.utils.RobotFuncs.intake
import org.firstinspires.ftc.teamcode.utils.RobotFuncs.log
import org.firstinspires.ftc.teamcode.utils.RobotFuncs.logs
import org.firstinspires.ftc.teamcode.utils.RobotVars.*
import java.util.Vector
import kotlin.Exception

class Clown(name: String) {
    private var USE_GHEARA_NEAR = true
    private var USE_GHEARA_FAR = true
    private var USE_GELENK = true

    val threads = Vector<Thread>()
    val RS = MServo(name + "RS", false, if (__IsAuto) DiffyDown / 2.0 + DiffyADown else if (USE_DIFFY) DiffyPrepDown / 2.0 + DiffyADown else null)
    val LS = MServo(name + "LS", true, if (__IsAuto) DiffyDown / 2.0 - DiffyADown else if (USE_DIFFY) DiffyPrepDown / 2.0 - DiffyADown else null)
    val RSE = AbsEnc(name + "RSE", DiffyEncROff)
    val LSE = AbsEnc(name + "LSE", DiffyEncLOff, true)
    val ghearaNear = if (USE_GHEARA_NEAR) MServo("GhearaNear", true, if (__IsAuto) ClownNInchis else if (USE_DIFFY) ClownNDeschis else null) else null
    val ghearaFar = if (USE_GHEARA_FAR) MServo("GhearaFar", false, if (__IsAuto) ClownFInchis else if (USE_DIFFY) ClownFDeschis else null) else null
    val gelenk = if (USE_GELENK) MServo("Gelenk", false, if (USE_DIFFY) GelenkCenter else null) else null

    private fun updateTarget() {
        if (USE_DIFFY) {
            RS.position = (targetPos / 2.0 + targetAngle) + DiffyROff
            LS.position = (targetPos / 2.0 - targetAngle) + DiffyLOff
        }
    }

    val actualDiffyPos: Double
        get() {
            //expansionHub.clearBulkCache()
            var rs = RSE.angle
            if (rs > 6.0) { rs = 0.0 }
            log("RSAng", rs)
            var ls = LSE.angle
            if (ls > 6.0) { ls = 0.0 }
            log("LSAng", ls)
            val curP = (rs + ls) / 2.0
            log("LSRes", curP)
            log("ACcessing diffy aaaaaaaaaaaaaa", etime.seconds())
            return curP
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
    var curState = -100

    private var sexPixelTraj = TrajectorySequence()
    private var goUpTraj = TrajectorySequence()
    private var goPreloadUpTraj = TrajectorySequence()
    private var goDownTraj = TrajectorySequence()
    private var goPreloadDownTraj = TrajectorySequence()

    val kmskm = ElapsedTime()


    init {
        run {
            goUpTraj.aa {
                targetPos = DiffyDown
                targetAngle = DiffyADown
                gelenk?.position = GelenkCenter
                log("GoUpTraj", 1)
            }
            goUpTraj.sl(ClownWait1)
            goUpTraj.aa {
                log("GoUpTraj", 2)
                close()
            }
            goUpTraj.sl(ClownWait2)
            goUpTraj.aa {
                log("GoUpTraj", 3)
                intake.status = SUpulLuiCostacu
            }
            goUpTraj.sl(ClownWait3)
            goUpTraj.aa {
                log("GoUpTraj", 4)
                curState = nextA
                targetPos = DiffyUp
            }
            //goUpTraj.aa { kmskm.reset() }
            //goUpTraj.wt { if (actualDiffyPos > DiffyWaitUpTurn) 1 else 0 }
            //goUpTraj.wt { kmskm.seconds() > 1.0 }
            goUpTraj.sl(ClownWait4)
            goUpTraj.aa {
                log("GoUpTraj", 5)
                gelenk?.position = GelenkCenter + curState * GelenkDif
                targetAngle = DiffyAUp
                targetPos = DiffyUp
            }
            /*
            goUpTraj.st(1)
            goUpTraj.aa { log("Prerepre", etime.seconds())}
            goUpTraj.gt {
                var curp = actualDiffyPos
                log("CurDiffyPos", curp)
                if (curp > DiffyWaitUpTurn) {
                    log ("Running into 1", etime.seconds())
                    1
                    //if () 2 else 1
                } else {
                    log ("Running into 2", etime.seconds())
                    2
                }
            }
            goUpTraj.st(2)
            goUpTraj.aa {
                log("GoUpTraj", 5)
                gelenk?.position = GelenkCenter + curState * GelenkDif
                targetAngle = DiffyAUp
            }*/
        } /// GoUp

        run {
            sexPixelTraj.aa {
                targetPos = DiffyDown
                targetAngle = DiffyADown
                gelenk?.position = GelenkCenter
            }
            sexPixelTraj.sl(ClownWait1)
            sexPixelTraj.aa {
                close()
            }
            sexPixelTraj.sl(ClownWait2)
            sexPixelTraj.aa {
                intake.status = SUpulLuiCostacu
            }
            sexPixelTraj.sl(ClownWait3)
            sexPixelTraj.aa {
                curState = -102
                targetPos = DiffyPrepDown
            }
        }

        run {
            goDownTraj.aa {
                open()
                curState = -100
                targetPos = DiffyPrepDown
                gelenk?.position = GelenkCenter
            }
            goDownTraj.sl(ClownWaitDown1)
            goDownTraj.aa {
                targetPos = DiffyPrepDown
                targetAngle = DiffyADown
                gelenk?.position = GelenkCenter
            }
        } /// Go Down

        run {
            /*goPreloadUpTraj.aa {
                targetPos = DiffyDown
                targetAngle = DiffyADown
                intake.status = SIntake
                gelenk?.position = GelenkCenter
            }
            goPreloadUpTraj.sl(ClownPWait1)*/
            goPreloadUpTraj.aa { close(); intake.status = SUpulLuiCostacu }
            goPreloadUpTraj.sl(ClownPWait2)
            goPreloadUpTraj.aa {
                targetPos = DiffyPreloadUp
            }
            goPreloadUpTraj.sl(ClownPWait3)
            goPreloadUpTraj.aa {
                targetAngle = DiffyAUp
            }
        } /// GoPreloadUp

        run {
            goPreloadDownTraj.aa {
                curState = -100
                targetPos = DiffyPrepDown
                gelenk?.position = GelenkCenter
                ghearaFar?.position = ClownFDeschis
            }
            goPreloadDownTraj.sl(ClownPWaitDown1)
            goPreloadDownTraj.aa { targetAngle = DiffyADown }
        } /// GoPreloadDown
    }

    private fun killextrathreads() {
        try {
            ___KILL_DIFFY_THREADS = true
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
            killextrathreads()
            when (curState) {
                -100 -> {
                    nextA = a
                    threads.add(goUpTraj.runAsyncDiffy())
                }
                -101, -102 -> {
                    gelenk?.position = GelenkCenter + curState * GelenkDif
                    targetAngle = DiffyAUp
                    targetPos = DiffyUp
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