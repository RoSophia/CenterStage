package org.firstinspires.ftc.teamcode.auto

import com.outoftheboxrobotics.photoncore.Photon
import com.qualcomm.robotcore.eventloop.opmode.Autonomous
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import org.firstinspires.ftc.teamcode.auto.MKMKMKMKMKMKM.NumCycles
import org.firstinspires.ftc.teamcode.pp.PP.JustDraw
import org.firstinspires.ftc.teamcode.utils.RobotFuncs.cam
import org.firstinspires.ftc.teamcode.utils.RobotFuncs.endma
import org.firstinspires.ftc.teamcode.utils.RobotFuncs.initAuto
import org.firstinspires.ftc.teamcode.utils.RobotFuncs.initma
import org.firstinspires.ftc.teamcode.utils.RobotFuncs.pp
import org.firstinspires.ftc.teamcode.utils.RobotFuncs.preinit
import org.firstinspires.ftc.teamcode.utils.RobotFuncs.startma
import org.firstinspires.ftc.teamcode.utils.RobotFuncs.update
import org.firstinspires.ftc.teamcode.utils.RobotVars.*

@Photon
@Autonomous
class BlueAuto : LinearOpMode() {
    private var targetPreload = 0
    override fun runOpMode() {
        preinit()
        initma(this)
        initAuto()
        while (!isStarted) {
            targetPreload = AutoResult
            telemetry.addData("TargetPreload", targetPreload)
            telemetry.update()
            sleep(5)
        }
        waitForStart()
        if (USE_CAMERA) {
            cam.stop()
        }
        startma()

        val ast = Cele10Traiectorii()
        var e = ast.getCycleTraj(NumCycles, false, targetPreload)

        while (!isStopRequested) {
            if (JustDraw) {
                e = ast.getCycleTraj(NumCycles, false, targetPreload)
                e.draw()
            } else {
                if (e.update()) {
                    requestOpModeStop()
                }
                pp.update()
            }
            update()
        }

        endma()
    }
}