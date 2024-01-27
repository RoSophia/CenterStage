package org.firstinspires.ftc.teamcode.auto

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import org.firstinspires.ftc.teamcode.pp.PP.JustDraw
import org.firstinspires.ftc.teamcode.utils.RobotFuncs
import org.firstinspires.ftc.teamcode.utils.RobotFuncs.cam
import org.firstinspires.ftc.teamcode.utils.RobotFuncs.endma
import org.firstinspires.ftc.teamcode.utils.RobotFuncs.initAuto
import org.firstinspires.ftc.teamcode.utils.RobotFuncs.initma
import org.firstinspires.ftc.teamcode.utils.RobotFuncs.localizer
import org.firstinspires.ftc.teamcode.utils.RobotFuncs.lom
import org.firstinspires.ftc.teamcode.utils.RobotFuncs.pp
import org.firstinspires.ftc.teamcode.utils.RobotFuncs.preinit
import org.firstinspires.ftc.teamcode.utils.RobotFuncs.startma
import org.firstinspires.ftc.teamcode.utils.RobotFuncs.update
import org.firstinspires.ftc.teamcode.utils.RobotVars
import org.firstinspires.ftc.teamcode.utils.RobotVars.AutoInitPos
import org.firstinspires.ftc.teamcode.utils.RobotVars.AutoRed
import org.firstinspires.ftc.teamcode.utils.RobotVars.CLOSE_IMU
import org.firstinspires.ftc.teamcode.utils.RobotVars.TIMMYOFF
import org.firstinspires.ftc.teamcode.utils.RobotVars.TOADKILL
import org.firstinspires.ftc.teamcode.utils.RobotVars.USE_CAMERA
import java.lang.Thread.sleep

object AutoFuncs {
    @JvmStatic
    var targetPreload = 0

    @JvmStatic
    private fun checkCamera(): Int {
        if (USE_CAMERA) {
            while (!lom.isStarted) {
                targetPreload = RobotVars.AutoResult
                RobotFuncs.log("TargetPreload", targetPreload)
                sleep(5)
            }
        }
        return targetPreload
    }

    @JvmStatic
    fun totBordu(lom: LinearOpMode, isRed: Boolean): Int {
        CLOSE_IMU = true
        AutoRed = isRed
        preinit()
        initma(lom)
        initAuto()
        val i = checkCamera()
        lom.waitForStart()
        if (USE_CAMERA) {
            cam.stop()
        }
        startma()
        TIMMYOFF = 0.0
        localizer.pose = AutoInitPos
        TOADKILL = true
        return i
    }

    @JvmStatic
    fun automa() {
        CLOSE_IMU = false
        endma()
    }

    @JvmStatic
    fun updateAuto(e: TrajectorySequence, numCycles: Int): TrajectorySequence? {
        if (JustDraw) {
            val ce = Cele10Traiectorii.getCycleTrajLongBlue(numCycles, targetPreload)
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
}