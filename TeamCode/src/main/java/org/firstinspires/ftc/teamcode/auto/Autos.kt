package org.firstinspires.ftc.teamcode.auto

import com.outoftheboxrobotics.photoncore.Photon
import com.qualcomm.robotcore.eventloop.opmode.Autonomous
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import org.firstinspires.ftc.teamcode.auto.AutoFuncs.setupAuto
import org.firstinspires.ftc.teamcode.auto.AutoVars.SLEEPY_TIME
import org.firstinspires.ftc.teamcode.auto.Cele10Traiectorii.getCycleFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFShort
import org.firstinspires.ftc.teamcode.auto.Cele10Traiectorii.getCycleTrajLong
import org.firstinspires.ftc.teamcode.auto.Cele10Traiectorii.getCycleTrajLongFFFFFFFFFFFFFFFFF
import org.firstinspires.ftc.teamcode.auto.Cele10Traiectorii.getCycleTrajShort
import org.firstinspires.ftc.teamcode.hardware.CameraControls.AutoResult

/*
@Autonomous(name = "CorrectRedLong", group = "CRed")
@Photon
class RedAuto: LinearOpMode() {
    override fun runOpMode() = setupAuto(this, isRed = true, isShort = false) { getCycleTrajLong(Auto.longRed) }
}

@Autonomous(name = "CorrectRedShort", group = "CRed")
@Photon
class RedShortAuto: LinearOpMode() {
    override fun runOpMode() { setupAuto(this, isRed = true, isShort = true) { getCycleTrajShort(Auto.shortRed) } }
}

 */

@Autonomous(name = "CorrectBlueLong SHAAAARK", group = "CBlue")
@Photon
class BlueAuto: LinearOpMode() {
    override fun runOpMode() {
        SLEEPY_TIME = 5.0
        setupAuto(this, isRed = false, isShort = false) { getCycleTrajLong(Auto.longBlue) }
    }
}

@Autonomous(name = "CorrectBlueLong Işí", group = "CBlue")
@Photon
class BlueAuto2: LinearOpMode() {
    override fun runOpMode() {
        SLEEPY_TIME = 2.0
        setupAuto(this, isRed = false, isShort = false) { getCycleTrajLong(Auto.longBlue) }
    }

}

/*
@Autonomous(name = "CorrectBlueShort", group = "CBlue")
@Photon
class BlueShortAuto: LinearOpMode() {
    override fun runOpMode() { setupAuto(this, isRed = false, isShort = true) { getCycleTrajShort(Auto.shortBlue) } }
}

@Autonomous(name = "IncorrectBlueLong", group = "ZBlue")
@Photon
class BlueLongSHITAuto: LinearOpMode() {
    override fun runOpMode() { setupAuto(this, isRed = false, isShort = false) { getCycleTrajLongFFFFFFFFFFFFFFFFF(Auto.longBlue) } }
}

@Autonomous(name = "IncorrectBlueShort", group = "ZBlue")
@Photon
class BlueShortSHITAuto: LinearOpMode() {
    override fun runOpMode() { setupAuto(this, isRed = false, isShort = true) { getCycleFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFShort(Auto.shortBlue) } }
}

@Autonomous(name = "IncorrectRedLong", group = "ZRed")
@Photon
class RedLongSHITAuto: LinearOpMode() {
    override fun runOpMode() { setupAuto(this, isRed = true, isShort = false) { getCycleTrajLongFFFFFFFFFFFFFFFFF(Auto.longRed) } }
}

@Autonomous(name = "IncorrectRedShort", group = "ZRed")
@Photon
class RedShortSHITAuto: LinearOpMode() {
    override fun runOpMode() { setupAuto(this, isRed = true, isShort = true) { getCycleFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFShort(Auto.shortRed) } }
}

 */
