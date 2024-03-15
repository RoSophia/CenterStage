package org.firstinspires.ftc.teamcode.auto

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import org.firstinspires.ftc.teamcode.hardware.CameraControls.AutoRed
import org.firstinspires.ftc.teamcode.hardware.CameraControls.AutoResult
import org.firstinspires.ftc.teamcode.pp.PP.JustDraw
import org.firstinspires.ftc.teamcode.utils.RobotFuncs
import org.firstinspires.ftc.teamcode.utils.RobotFuncs.cam
import org.firstinspires.ftc.teamcode.utils.RobotFuncs.create_god
import org.firstinspires.ftc.teamcode.utils.RobotFuncs.endma
import org.firstinspires.ftc.teamcode.utils.RobotFuncs.initAuto
import org.firstinspires.ftc.teamcode.utils.RobotFuncs.initma
import org.firstinspires.ftc.teamcode.utils.RobotFuncs.log
import org.firstinspires.ftc.teamcode.utils.RobotFuncs.lom
import org.firstinspires.ftc.teamcode.utils.RobotFuncs.pp
import org.firstinspires.ftc.teamcode.utils.RobotFuncs.preinit
import org.firstinspires.ftc.teamcode.utils.RobotFuncs.startma
import org.firstinspires.ftc.teamcode.utils.RobotFuncs.update
import org.firstinspires.ftc.teamcode.utils.RobotVars.TimmyToClose
import org.firstinspires.ftc.teamcode.utils.RobotVars.TimmyCurOff
import org.firstinspires.ftc.teamcode.utils.RobotVars.TimmyAddKILLLLLLLL
import org.firstinspires.ftc.teamcode.utils.RobotVars.USE_CAMERA
import org.firstinspires.ftc.teamcode.utils.RobotVars.__AutoShort
import java.lang.Thread.sleep
import kotlin.math.PI

object AutoFuncs {

    private fun updateAuto(e: TrajectorySequence, func: ()->TrajectorySequence): TrajectorySequence? {
        if (JustDraw) {
            val ce = func()
            ce.draw()
            update()
            return ce
        } else {
            if (e.update()) {
                lom.requestOpModeStop()
            }
            pp.update()
        }
        update()
        return null
    }

    fun setupAuto(lom: LinearOpMode, isRed: Boolean, isShort: Boolean, func: ()->TrajectorySequence) {
        totBordu(lom, isRed, isShort)
        var e = func()

        while (!lom.isStopRequested) {
            val ce = updateAuto(e, func)
            if (ce != null) { e = ce }
        }

        TimmyToClose = false
        TimmyCurOff = PI
        endma()
    }

    fun totBordu(lom: LinearOpMode, isRed: Boolean, isShort: Boolean): Int {
        TimmyToClose = true
        AutoRed = isRed
        __AutoShort = isShort
        preinit()
        initma(lom, true)
        initAuto()
        lom.waitForStart()
        val res = AutoResult // Pun asta aici sa nu faca amuzant inchiderea camerei
        if (USE_CAMERA) { cam.stop() }
        startma()
        TimmyCurOff = 0.0
        TimmyAddKILLLLLLLL = true
        return res
    }

    fun updateAutoCustom(e: TrajectorySequence, func: ()->TrajectorySequence): TrajectorySequence? {
        if (JustDraw) { val ce = func(); ce.draw(); update(); return ce }
        else {
            if (e.update()) { return func() }
            pp.update()
        }
        update()
        return null
    }
}