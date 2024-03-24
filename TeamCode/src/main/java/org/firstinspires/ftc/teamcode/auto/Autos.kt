package org.firstinspires.ftc.teamcode.auto

import com.outoftheboxrobotics.photoncore.Photon
import com.qualcomm.robotcore.eventloop.opmode.Autonomous
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import org.firstinspires.ftc.teamcode.auto.AutoFuncs.setupAuto
import org.firstinspires.ftc.teamcode.auto.AutoVars.NumCycles
import org.firstinspires.ftc.teamcode.auto.Cele10Traiectorii.getCycleTrajLong
import org.firstinspires.ftc.teamcode.auto.Cele10Traiectorii.getCycleTrajShort
import org.firstinspires.ftc.teamcode.hardware.CameraControls.AutoResult
import org.firstinspires.ftc.teamcode.utils.RobotVars.ShortPlus

//@Autonomous
@Photon
class RedAuto: LinearOpMode() {
    override fun runOpMode() = setupAuto(this, isRed = true, isShort = false) { getCycleTrajLong(NumCycles, AutoResult, Auto.longRed) }
}

//@Autonomous
@Photon
class RedShortAuto: LinearOpMode() {
    override fun runOpMode() = setupAuto(this, isRed = true, isShort = true) { getCycleTrajShort(NumCycles, AutoResult, Auto.shortRed) }
}

//@Autonomous
@Photon
class BlueAuto: LinearOpMode() {
    override fun runOpMode() = setupAuto(this, isRed = false, isShort = false) { getCycleTrajLong(NumCycles, AutoResult, Auto.longBlue) }
}

//@Autonomous
@Photon
class BlueShortAuto: LinearOpMode() {
    override fun runOpMode() {
        ShortPlus = false
        setupAuto(this, isRed = false, isShort = true) { getCycleTrajShort(NumCycles, AutoResult, Auto.shortBlue) }
    }
}

//@Autonomous
@Photon
class BlueShortAutoBig: LinearOpMode() {
    override fun runOpMode() {
        ShortPlus = true
        setupAuto(this, isRed = false, isShort = true) { getCycleTrajShort(NumCycles, AutoResult, Auto.shortBlue) }
    }
}

@Autonomous
@Photon
class RedShortAutoBig: LinearOpMode() {
    override fun runOpMode() {
        ShortPlus = true
        setupAuto(this, isRed = true, isShort = true) { getCycleTrajShort(NumCycles, AutoResult, Auto.shortRed) }
    }
}
