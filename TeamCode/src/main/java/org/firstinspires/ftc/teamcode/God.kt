package org.firstinspires.ftc.teamcode

import com.outoftheboxrobotics.photoncore.Photon
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import com.qualcomm.robotcore.hardware.Gamepad
import com.qualcomm.robotcore.util.ElapsedTime
import org.firstinspires.ftc.teamcode.auto.TrajectorySequence
import org.firstinspires.ftc.teamcode.utils.RobotFuncs
import org.firstinspires.ftc.teamcode.utils.RobotFuncs.KILLALL
import org.firstinspires.ftc.teamcode.utils.RobotFuncs.create_god
import org.firstinspires.ftc.teamcode.utils.RobotFuncs.log
import org.firstinspires.ftc.teamcode.utils.RobotFuncs.preinit
import org.firstinspires.ftc.teamcode.utils.RobotFuncs.send_log
import org.firstinspires.ftc.teamcode.utils.RobotVars.KILLMYSELF

@Photon
@TeleOp
class CreateGod : LinearOpMode() {
    override fun runOpMode() {
        preinit()
        RobotFuncs.lom = this
        RobotFuncs.etime.reset()

        create_god()

        val etime = ElapsedTime()
        var kms = 0
        etime.reset()
        val t = TrajectorySequence()
                .st(2)
                .sl(0.6)
                .aa { ++kms }
                .sl(0.6)
                .gt { 2 }
                .runAsync()

        while (!isStopRequested) {
            log("ISAL", t.isAlive)
            log("KMS", kms)
            send_log()
            sleep(10)
        }

        //gamepad1.runLedEffect(Gamepad.LedEffect.Builder().addStep(0.0, 1.0, 0.0, 300).addStep(1.0, 1.0, 1.0, 300).setRepeating(true).build())

        waitForStart()
    }
}
