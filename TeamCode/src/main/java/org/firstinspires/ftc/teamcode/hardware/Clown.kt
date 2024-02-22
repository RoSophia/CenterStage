package org.firstinspires.ftc.teamcode.hardware

import org.firstinspires.ftc.teamcode.auto.TrajectorySequence
import org.firstinspires.ftc.teamcode.hardware.Intakes.SUpulLuiCostacu
import org.firstinspires.ftc.teamcode.utils.RobotFuncs.etime
import org.firstinspires.ftc.teamcode.utils.RobotFuncs.intake
import org.firstinspires.ftc.teamcode.utils.RobotFuncs.log
import org.firstinspires.ftc.teamcode.utils.RobotVars.ClownFDeschis
import org.firstinspires.ftc.teamcode.utils.RobotVars.ClownFInchis
import org.firstinspires.ftc.teamcode.utils.RobotVars.ClownNDeschis
import org.firstinspires.ftc.teamcode.utils.RobotVars.ClownNInchis
import org.firstinspires.ftc.teamcode.utils.RobotVars.ClownWait1
import org.firstinspires.ftc.teamcode.utils.RobotVars.ClownWait2
import org.firstinspires.ftc.teamcode.utils.RobotVars.ClownWait3
import org.firstinspires.ftc.teamcode.utils.RobotVars.ClownWaitDown1
import org.firstinspires.ftc.teamcode.utils.RobotVars.ClownWaitDown2
import org.firstinspires.ftc.teamcode.utils.RobotVars.GelenkDif
import org.firstinspires.ftc.teamcode.utils.RobotVars.DiffyADown
import org.firstinspires.ftc.teamcode.utils.RobotVars.DiffyAUp
import org.firstinspires.ftc.teamcode.utils.RobotVars.DiffyDown
import org.firstinspires.ftc.teamcode.utils.RobotVars.DiffyLOff
import org.firstinspires.ftc.teamcode.utils.RobotVars.DiffyPrepDown
import org.firstinspires.ftc.teamcode.utils.RobotVars.DiffyROff
import org.firstinspires.ftc.teamcode.utils.RobotVars.DiffyUp
import org.firstinspires.ftc.teamcode.utils.RobotVars.GelenkCenter
import org.firstinspires.ftc.teamcode.utils.RobotVars.USE_DIFFY
import java.util.Vector

class Clown(name: String) {
    val threads = Vector<Thread>()
    val RS = MServo(name + "RS", false, DiffyPrepDown / 2.0 + DiffyADown)
    val LS = MServo(name + "LS", true, DiffyPrepDown / 2.0 - DiffyADown)
    val ghearaNear = MServo("GhearaNear", true, ClownNDeschis)
    val ghearaFar = MServo("GhearaFar", false, ClownFDeschis)
    val gelenk = MServo("Gelenk", false, GelenkCenter)

    private fun updateTarget() {
        if (USE_DIFFY) {
            RS.position = (targetPos / 2.0 + targetAngle) + DiffyROff
            LS.position = (targetPos / 2.0 - targetAngle) + DiffyLOff
        }
    }

    private fun updateAngle() {
        if (USE_DIFFY) {
            gelenk.position = GelenkCenter + curAngle * GelenkDif
            targetAngle = DiffyAUp
        }
    }

    var curAngle = -100

    private var goUpTraj = TrajectorySequence()
    private var goDownTraj = TrajectorySequence()

    init {
        goUpTraj.aa {
            targetPos = DiffyDown
            targetAngle = DiffyADown
            gelenk.position = GelenkCenter
            intake.status = SUpulLuiCostacu
        }
        goUpTraj.sl(ClownWait1)
        goUpTraj.aa { close() }
        goUpTraj.sl(ClownWait2)
        goUpTraj.aa {
            curAngle = nextA
            targetPos = DiffyUp
        }
        goUpTraj.sl(ClownWait3)
        goUpTraj.aa {
            gelenk.position = GelenkCenter + curAngle * GelenkDif
            targetAngle = DiffyAUp
        }

        goDownTraj.aa {
            curAngle = -100
            targetPos = DiffyPrepDown
            gelenk.position = GelenkCenter
        }
        goDownTraj.sl(ClownWaitDown1)
        goDownTraj.aa { targetAngle = DiffyADown }
        goDownTraj.sl(ClownWaitDown2)
        goDownTraj.aa { open() }
    }

    private fun killextrathreads() {
        for (t in threads) {
            if (t.isAlive) {
                log("KIlling thread ${t.id} at", etime.seconds())
                t.interrupt()
            }
        }
        threads.clear()
    }

    fun goLeft() {
        killextrathreads()
        if (curAngle == -100) {
            goUp(-1)
        } else {
            if (curAngle > -2) {
                --curAngle
                updateAngle()
            }
        }
    }

    fun goRight() {
        killextrathreads()
        if (curAngle == -100) {
            goUp(1)
        } else {
            if (curAngle < 2) {
                ++curAngle
                updateAngle()
            }
        }
    }

    fun close() {
        ghearaNear.position = ClownNInchis
        ghearaFar.position = ClownFInchis
    }

    fun open() {
        ghearaNear.position = ClownNDeschis
        ghearaFar.position = ClownFDeschis
    }

    var nextA = 0

    fun goUp(a: Int) { ////// TODO:    REMOVEMOMOVEMOVE
        killextrathreads()
        ////// TODO:    REMOVEMOMOVEMOVE ////// TODO:    REMOVEMOMOVEMOVE ////// TODO:    REMOVEMOMOVEMOVE ////// TODO:    REMOVEMOMOVEMOVE

        ////// TODO:    REMOVEMOMOVEMOVE
        ////// TODO:    REMOVEMOMOVEMOVE
        goUpTraj = TrajectorySequence()
        goUpTraj.aa {
            targetPos = DiffyDown
            targetAngle = DiffyADown
            gelenk.position = GelenkCenter
            intake.status = SUpulLuiCostacu
        }
        goUpTraj.sl(ClownWait1)
        goUpTraj.aa { close() }
        goUpTraj.sl(ClownWait2)
        goUpTraj.aa {
            curAngle = nextA
            targetPos = DiffyUp
        }
        goUpTraj.sl(ClownWait3)
        goUpTraj.aa {
            gelenk.position = GelenkCenter + curAngle * GelenkDif
            targetAngle = DiffyAUp
        }
        if (curAngle == -100) {
            nextA = a
            killextrathreads()
            threads.add(goUpTraj.runAsync())
        } else {
            curAngle = a
            updateAngle()
        }
    }

    fun goDown() {
        killextrathreads()
        threads.add(goDownTraj.runAsync())
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