package org.firstinspires.ftc.teamcode

import com.outoftheboxrobotics.photoncore.Photon
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import com.qualcomm.robotcore.eventloop.opmode.TeleOp

@Photon(maximumParallelCommands = 10)
@TeleOp(name = "我討厭修訂 Cu Pid")
class OpIHATEREV_CuPid : LinearOpMode() {
    override fun runOpMode() {
        tilipo.runOpMode(this, true)
    }
}