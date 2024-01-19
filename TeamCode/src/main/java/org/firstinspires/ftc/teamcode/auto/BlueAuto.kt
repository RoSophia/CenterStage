package org.firstinspires.ftc.teamcode.auto

import com.outoftheboxrobotics.photoncore.Photon
import com.qualcomm.robotcore.eventloop.opmode.Autonomous
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import org.firstinspires.ftc.teamcode.hardware.Intakes
import org.firstinspires.ftc.teamcode.pp.PP.JustDraw
import org.firstinspires.ftc.teamcode.utils.RobotFuncs.cam
import org.firstinspires.ftc.teamcode.utils.RobotFuncs.controller
import org.firstinspires.ftc.teamcode.utils.RobotFuncs.initAuto
import org.firstinspires.ftc.teamcode.utils.RobotFuncs.initma
import org.firstinspires.ftc.teamcode.utils.RobotFuncs.intake
import org.firstinspires.ftc.teamcode.utils.RobotFuncs.localizer
import org.firstinspires.ftc.teamcode.utils.RobotFuncs.log
import org.firstinspires.ftc.teamcode.utils.RobotFuncs.moveSwerve
import org.firstinspires.ftc.teamcode.utils.RobotFuncs.pp
import org.firstinspires.ftc.teamcode.utils.RobotFuncs.preinit
import org.firstinspires.ftc.teamcode.utils.RobotFuncs.startma
import org.firstinspires.ftc.teamcode.utils.RobotVars

@Photon
@Autonomous
class BlueAuto : LinearOpMode() {
    var targetPreload = 0
    override fun runOpMode() {
        preinit()
        initma(this)
        initAuto()
        intake.status = Intakes.SDown
        while (!isStarted) {
            targetPreload = RobotVars.AutoResult
            telemetry.addData("TargetPreload", targetPreload)
            telemetry.update()
            sleep(5)
        }
        waitForStart()
        cam.stop()
        startma()

        /*
        val ast = Cele10Traiectorii()
        var e = ast.getCycleTraj(0, false, targetPreload)

        while (!isStopRequested) {
            if (JustDraw) {
                e = ast.getCycleTraj(0, false, targetPreload)
                e.draw()
            } else {
                updateTraj()
                pp.update()
            }
            update()
        }

        endma()*/
    }
}