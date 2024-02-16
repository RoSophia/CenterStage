package org.firstinspires.ftc.teamcode.auto

import com.outoftheboxrobotics.photoncore.Photon
import com.qualcomm.robotcore.eventloop.opmode.Autonomous
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import org.firstinspires.ftc.teamcode.auto.AutoFuncs.automa
import org.firstinspires.ftc.teamcode.auto.AutoVars.NumCycles
import org.firstinspires.ftc.teamcode.utils.RobotVars.AutoResult

@Autonomous
@Photon
class BlueAutoShort : LinearOpMode() {
    override fun runOpMode() {
        AutoFuncs.totBordu(this, false)
        var e = Cele10Traiectorii.getCycleTrajShortBlue(NumCycles, AutoResult)

        while (!isStopRequested) {
            val ce = AutoFuncs.updateAuto(e) { Cele10Traiectorii.getCycleTrajShortBlue(NumCycles, AutoResult) }
            if (ce != null) {
                e = ce
            }
        }

        automa()
    }
}