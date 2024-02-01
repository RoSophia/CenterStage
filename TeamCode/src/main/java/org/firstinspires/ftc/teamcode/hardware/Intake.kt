package org.firstinspires.ftc.teamcode.hardware

import com.qualcomm.robotcore.util.ElapsedTime
import org.firstinspires.ftc.teamcode.hardware.Intakes.SDown
import org.firstinspires.ftc.teamcode.hardware.Intakes.SGoDownForOuttake
import org.firstinspires.ftc.teamcode.hardware.Intakes.SIntake
import org.firstinspires.ftc.teamcode.hardware.Intakes.SInvert
import org.firstinspires.ftc.teamcode.hardware.Intakes.SNothing
import org.firstinspires.ftc.teamcode.hardware.Intakes.SPStack1
import org.firstinspires.ftc.teamcode.hardware.Intakes.SPStack2
import org.firstinspires.ftc.teamcode.hardware.Intakes.SPStack3
import org.firstinspires.ftc.teamcode.hardware.Intakes.SReset2
import org.firstinspires.ftc.teamcode.hardware.Intakes.SStack1
import org.firstinspires.ftc.teamcode.hardware.Intakes.SStack2
import org.firstinspires.ftc.teamcode.hardware.Intakes.SStack3
import org.firstinspires.ftc.teamcode.hardware.Intakes.SWaitFor
import org.firstinspires.ftc.teamcode.utils.RobotVars.*
import org.firstinspires.ftc.teamcode.utils.Util.epsEq

object Intakes {
    const val SNothing = 0
    const val SIntake = 1
    const val SStack1 = 2
    const val SStack2 = 3
    const val SStack3 = 4
    const val SWaitFor = 5
    const val SPStack1 = 6
    const val SPStack2 = 7
    const val SPStack3 = 8
    const val SReset = 9
    const val SReset2 = 10
    const val SGoDownForOuttake = 11
    const val SInvert = 12
    const val SDown = 13
}

class Intake {
    private val intake = Motor("Intake", encoder = false, rev = false, overdrive = true)
    private val ridIntake1 = MServo("RidIntake", IntakeP1Up)
    private val ridIntake2 = MServo("SoftStage2VericuRupeAderenta", IntakeP2Up)

    private var cstack = 0

    val running: Boolean
        get() = !epsEq(intake.power, 0.0)

    private val sttimer = ElapsedTime()
    var status = 0
        set(v) {
            when (v) {
                SNothing -> {
                    ridIntake1.position = IntakeP1Up
                    ridIntake2.position = IntakeP2Up
                    intake.power = 0.0
                }

                SDown -> {
                    ridIntake1.position = IntakeP1Down
                    ridIntake2.position = IntakeP2Down
                    intake.power = 0.0
                }

                SIntake -> {
                    ridIntake1.position = IntakeP1Down
                    ridIntake2.position = IntakeP2Down
                    intake.power = IntakePower
                }

                SStack1 -> {
                    ridIntake1.position = IntakeP1Stack1
                    intake.power = IntakePowerStack
                }

                SPStack1 -> {
                    ridIntake1.position = IntakeP1Stack1 + IntakePrepDif / 2.0
                    cstack = 1
                    intake.power = 0.0
                }

                SStack2 -> {
                    ridIntake1.position = IntakeP1Stack2
                    intake.power = IntakePowerStack
                }

                SPStack2 -> {
                    ridIntake1.position = IntakeP1Stack2 + IntakePrepDif
                    cstack = 2
                    intake.power = 0.0
                }

                SStack3 -> {
                    ridIntake1.position = IntakeP1Stack3
                    intake.power = IntakePower
                }

                SPStack3 -> {
                    ridIntake1.position = IntakeP1Stack3 + IntakePrepDif
                    cstack = 3
                    intake.power = 0.0
                }

                SReset2 -> {
                    ridIntake1.position = IntakeP1Up
                    intake.power = 0.0
                }

                SInvert -> {
                    ridIntake1.position = IntakeP1Up
                    ridIntake2.position = IntakeP2Up
                    intake.power = -IntakePower
                }

                SGoDownForOuttake -> {
                    ridIntake1.position = IntakeP1Down
                    intake.power = 0.0
                }

                else -> {}
            }
            field = v
            if (v == SPStack1 || v == SPStack2 || v == SPStack3) {
                sttimer.reset()
            }
        }

    private val intakeTimer = ElapsedTime()
    private var lastStatus = SNothing
    fun update() {
        if (status == SReset2) {
            if (intakeTimer.seconds() > IntakeWaitTime) {
                status = lastStatus
            }
        } else if (status == SWaitFor) {
            if (sttimer.seconds() > IntakeWaitFallTime) {
                status = if (cstack == 1) SStack1 else if (cstack == 2) SStack2 else SStack3
            }
        }
    }
}