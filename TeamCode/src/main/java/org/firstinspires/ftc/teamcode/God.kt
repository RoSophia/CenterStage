package org.firstinspires.ftc.teamcode

import com.outoftheboxrobotics.photoncore.Photon
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import com.qualcomm.robotcore.util.ElapsedTime
import org.firstinspires.ftc.teamcode.auto.TrajectorySequence
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
        /*
        val ep = ElapsedTime()
        val tr = TrajectorySequence()
        tr.st(1)
        tr.aa { log("Cycle", ep.seconds()) }
        tr.gt { if (KILLMYSELF) 1 else 2 }
        tr.st(2)
        tr.aa { log("Done", ep.seconds()) }
        while (tr.update()) {
            send_log()
        }*/


        create_god()
    }
}
