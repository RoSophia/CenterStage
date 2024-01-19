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
import org.firstinspires.ftc.teamcode.hardware.Intakes.SReset
import org.firstinspires.ftc.teamcode.hardware.Intakes.SReset2
import org.firstinspires.ftc.teamcode.hardware.Intakes.SStack1
import org.firstinspires.ftc.teamcode.hardware.Intakes.SStack2
import org.firstinspires.ftc.teamcode.hardware.Intakes.SStack3
import org.firstinspires.ftc.teamcode.hardware.Intakes.SWaitFor
import org.firstinspires.ftc.teamcode.utils.RobotFuncs
import org.firstinspires.ftc.teamcode.utils.RobotFuncs.log
import org.firstinspires.ftc.teamcode.utils.RobotVars
import org.firstinspires.ftc.teamcode.utils.RobotVars.IntakeMaxCurrent
import org.firstinspires.ftc.teamcode.utils.RobotVars.IntakeMaxCurrentTime
import org.firstinspires.ftc.teamcode.utils.RobotVars.IntakePDown
import org.firstinspires.ftc.teamcode.utils.RobotVars.IntakePStack1
import org.firstinspires.ftc.teamcode.utils.RobotVars.IntakePStack2
import org.firstinspires.ftc.teamcode.utils.RobotVars.IntakePStack3
import org.firstinspires.ftc.teamcode.utils.RobotVars.IntakePUp
import org.firstinspires.ftc.teamcode.utils.RobotVars.IntakePower
import org.firstinspires.ftc.teamcode.utils.RobotVars.IntakeWaitFallTime
import org.firstinspires.ftc.teamcode.utils.RobotVars.IntakeWaitTime
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
    val intake = Motor("Intake", encoder = false, rev = false, overdrive = true)
    val ridIntake = MServo("RidIntake", IntakePUp)

    val running: Boolean
        get() = !epsEq(intake.power, 0.0)

    val sttimer = ElapsedTime()
    var status = 0
        set(v) {
            if (v == SNothing) {
                ridIntake.position = IntakePUp
                intake.power = 0.0
            }
            if (v == SDown) {
                ridIntake.position = IntakePDown
                intake.power = 0.0
            }
            if (v == SIntake) {
                ridIntake.position = IntakePDown
                intake.power = IntakePower
            }
            if (v == SStack1 || v == SPStack1) {
                ridIntake.position = IntakePStack1
            }
            if (v == SStack2 || v == SPStack2) {
                ridIntake.position = IntakePStack2
            }
            if (v == SStack3 || v == SPStack3) {
                ridIntake.position = IntakePStack3
            }
            if (v == SReset2) {
                intake.power = 0.0
                ridIntake.position = IntakePUp
            }
            if (v == SInvert) {
                ridIntake.position = IntakePDown
                intake.power = -IntakePower
            }
            if (v == SGoDownForOuttake) {
                ridIntake.position = IntakePDown
                intake.power = 0.0
            }
            field = v
            if (v == SStack1 || v == SStack2 || v == SStack3) {
                intake.power = IntakePower
                field = SWaitFor
                sttimer.reset()
            }
        }

    val intakeTimer = ElapsedTime()
    var lastStatus = SNothing
    fun update() {
        if (status == SReset2) {
            if (intakeTimer.seconds() > IntakeWaitTime) {
                status = lastStatus
            }
        } else if (status == SWaitFor) {
            if (sttimer.seconds() > IntakeWaitFallTime) {
                status = SIntake
            }
        }
    }
}