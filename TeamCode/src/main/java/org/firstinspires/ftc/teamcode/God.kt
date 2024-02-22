package org.firstinspires.ftc.teamcode

import com.outoftheboxrobotics.photoncore.Photon
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import org.firstinspires.ftc.teamcode.utils.RobotFuncs.create_god

@Photon
@TeleOp
class CreateGod : LinearOpMode() {
    override fun runOpMode() {
        create_god()
    }
}
