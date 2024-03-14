package org.firstinspires.ftc.teamcode.auto

import com.outoftheboxrobotics.photoncore.Photon
import com.qualcomm.robotcore.eventloop.opmode.Autonomous
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import org.firstinspires.ftc.teamcode.auto.AutoFuncs.setupAuto
import org.firstinspires.ftc.teamcode.auto.AutoVars.NumCycles
import org.firstinspires.ftc.teamcode.auto.Cele10Traiectorii.getCycleTrajLong
import org.firstinspires.ftc.teamcode.auto.Cele10Traiectorii.getCycleTrajShort
import org.firstinspires.ftc.teamcode.hardware.CameraControls.AutoResult

@Autonomous
@Photon
class RedAuto: LinearOpMode() {
    override fun runOpMode() = setupAuto(this, isRed = true, isShort = false) { getCycleTrajLong(NumCycles, AutoResult, Longies.red) }
}

@Autonomous
@Photon
class RedShortAuto: LinearOpMode() {
    override fun runOpMode() = setupAuto(this, isRed = true, isShort = true) { getCycleTrajShort(NumCycles, AutoResult, Shorties.red) }
}

@Autonomous
@Photon
class BlueAuto: LinearOpMode() {
    override fun runOpMode() = setupAuto(this, isRed = false, isShort = false) { getCycleTrajLong(NumCycles, AutoResult, Longies.blue) }
}

@Autonomous
@Photon
class BlueShortAuto: LinearOpMode() {
    override fun runOpMode() = setupAuto(this, isRed = false, isShort = true) { getCycleTrajShort(NumCycles, AutoResult, Shorties.blue) }
}
