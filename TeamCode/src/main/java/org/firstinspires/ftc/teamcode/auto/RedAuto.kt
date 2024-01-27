package org.firstinspires.ftc.teamcode.auto

import com.outoftheboxrobotics.photoncore.Photon
import com.qualcomm.robotcore.eventloop.opmode.Autonomous
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import org.firstinspires.ftc.teamcode.auto.AutoFuncs.automa
import org.firstinspires.ftc.teamcode.auto.AutoFuncs.totBordu
import org.firstinspires.ftc.teamcode.auto.AutoVars.NumCycles
import org.firstinspires.ftc.teamcode.utils.RobotVars.AutoResult

@Autonomous
@Photon
class RedAuto: LinearOpMode() {
    override fun runOpMode() {
        totBordu(this, true)
        var e = Cele10Traiectorii.getCycleTrajLongRed(NumCycles, AutoResult)

        while (!isStopRequested) {
            val ce = AutoFuncs.updateAuto(e, NumCycles)
            if (ce != null) {
                e = ce
            }
        }

        automa()
    }
}