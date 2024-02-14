package org.firstinspires.ftc.teamcode.hardware

import org.firstinspires.ftc.teamcode.hardware.Intakes.SDown
import org.firstinspires.ftc.teamcode.hardware.Intakes.SIdleIntake
import org.firstinspires.ftc.teamcode.hardware.Intakes.SIntake
import org.firstinspires.ftc.teamcode.hardware.Intakes.SInvert
import org.firstinspires.ftc.teamcode.hardware.Intakes.SNothing
import org.firstinspires.ftc.teamcode.hardware.Intakes.SPStack1
import org.firstinspires.ftc.teamcode.hardware.Intakes.SPStack2
import org.firstinspires.ftc.teamcode.hardware.Intakes.SPStack3
import org.firstinspires.ftc.teamcode.hardware.Intakes.SStack1
import org.firstinspires.ftc.teamcode.hardware.Intakes.SStack2
import org.firstinspires.ftc.teamcode.hardware.Intakes.SStack3
import org.firstinspires.ftc.teamcode.hardware.Intakes.SUp
import org.firstinspires.ftc.teamcode.utils.RobotVars.*
import org.firstinspires.ftc.teamcode.utils.Util.epsEq
import org.firstinspires.ftc.teamcode.utils.Vec4

object Intakes {
    const val SNothing = 0
    const val SIntake = 1
    const val SStack1 = 2
    const val SStack2 = 3
    const val SStack3 = 4
    const val SPStack1 = 6
    const val SPStack2 = 7
    const val SPStack3 = 8
    const val SIdleIntake = 9
    const val SInvert = 12
    const val SDown = 13
    const val SUp = 14
}

class Intake {
    private val intake = Motor("Intake", encoder = false, rev = false, overdrive = true)
    private val ridIntake1 = MServo("RidIntake")
    private val ridIntake2 = MServo("SoftStage2VericuRupeAderenta")

    val running: Boolean
        get() = !epsEq(intake.power, 0.0)

    private fun sint(kms: Vec4, p: Int, pwr: Double) {
        ridIntake1.position = kms[p * 2 + 0]
        ridIntake2.position = kms[p * 2 + 1]
        intake.power = pwr
    }

    private fun sint(kms: Vec4, p: Int) = sint(kms, p, IntakePower)

    private fun sint(p1: Double, p2: Double, pwr: Double) {
        ridIntake1.position = p1
        ridIntake2.position = p2
        intake.power = pwr
    }

    var status = 0
        set(v) {
            if (USE_INTAKE) {
                when (v) {
                    SNothing -> sint(IntakeGet, 1, 0.0)
                    SDown -> sint(IntakeGetUp, 0, 0.0)
                    SUp -> sint(IntakeGetUp, 1, 0.0)
                    SIntake -> sint(IntakeGet, 0, IntakePower)
                    SStack1 -> sint(IntakeStack1, 1)
                    SPStack1 -> sint(IntakeStack1, 0)
                    SStack2 -> sint(IntakeStack2, 1)
                    SPStack2 -> sint(IntakeStack2, 0)
                    SStack3 -> sint(IntakeStack3, 1)
                    SPStack3 -> sint(IntakeStack3, 0)
                    SInvert -> sint(IntakeGet, 1, IntakeOuttakePower)
                    SIdleIntake -> sint(IntakeGetUp, 0)
                    else -> {}
                }
                field = v
            }
        }

    init {
        status = SNothing
    }
}