package org.firstinspires.ftc.teamcode.hardware

import com.qualcomm.robotcore.util.ElapsedTime
import org.firstinspires.ftc.teamcode.hardware.Intakes.SDown
import org.firstinspires.ftc.teamcode.hardware.Intakes.SFinalHang
import org.firstinspires.ftc.teamcode.hardware.Intakes.SIdleIntake
import org.firstinspires.ftc.teamcode.hardware.Intakes.SIntake
import org.firstinspires.ftc.teamcode.hardware.Intakes.SInvert
import org.firstinspires.ftc.teamcode.hardware.Intakes.SInvert2
import org.firstinspires.ftc.teamcode.hardware.Intakes.SKeep
import org.firstinspires.ftc.teamcode.hardware.Intakes.SNothing
import org.firstinspires.ftc.teamcode.hardware.Intakes.SStack1
import org.firstinspires.ftc.teamcode.hardware.Intakes.SStack2
import org.firstinspires.ftc.teamcode.hardware.Intakes.SStack3
import org.firstinspires.ftc.teamcode.hardware.Intakes.SStack4
import org.firstinspires.ftc.teamcode.hardware.Intakes.SStack5
import org.firstinspires.ftc.teamcode.hardware.Intakes.SStack6
import org.firstinspires.ftc.teamcode.hardware.Intakes.SUp
import org.firstinspires.ftc.teamcode.hardware.Intakes.SUpulLuiCostacu
import org.firstinspires.ftc.teamcode.hardware.Intakes.SUpulLuiCostacuIn
import org.firstinspires.ftc.teamcode.utils.RobotFuncs.clown
import org.firstinspires.ftc.teamcode.utils.RobotFuncs.etime
import org.firstinspires.ftc.teamcode.utils.RobotFuncs.log
import org.firstinspires.ftc.teamcode.utils.RobotFuncs.logs
import org.firstinspires.ftc.teamcode.utils.RobotFuncs.lom
import org.firstinspires.ftc.teamcode.utils.RobotVars.*
import org.firstinspires.ftc.teamcode.utils.Util.epsEq
import org.firstinspires.ftc.teamcode.utils.Vec4

object Intakes {
    const val SNothing = 0
    const val SIntake = 1
    const val SStack1 = 2
    const val SStack2 = 3
    const val SStack3 = 4
    const val SStack4 = 6
    const val SStack5 = 7
    const val SStack6 = 8
    const val SIdleIntake = 9
    const val SInvert = 12
    const val SDown = 13
    const val SUp = 14
    const val SUpulLuiCostacu = 15
    const val SUpulLuiCostacuIn = 16
    const val SKeep = 17
    const val SInvert2 = 18
    const val SFinalHang = 19
}

class Intake {
    val intake = Motor("Intake", encoder = false, rev = false, overdrive = true)
    private val ridIntake1 = MServo("RidIntake")
    private val ridIntake2 = MServo("RidIntakeFar")

    val running: Boolean get() = !epsEq(intake.power, 0.0)

    private fun sint(kms: Vec4, p: Int, pwr: Double) { ridIntake1.position = kms[p * 2 + 0]; ridIntake2.position = kms[p * 2 + 1]; intake.power = pwr }
    private fun sint(kms: Vec4, p: Int) = sint(kms, p, IntakePower)
    private fun sint(p1: Double, p2: Double, pwr: Double) { ridIntake1.position = p1; ridIntake2.position = p2; intake.power = pwr }

    private var checkVibr = false
    private var vibeTime = ElapsedTime()
    fun update() {
        if (checkVibr && __UPDATE_SENSORS) {
            val ro = clown.sensorReadout()
            log("GotReadout", "$ro at ${etime.seconds()}")
            if (ro == 3) {
                if (vibeTime.seconds() > 0.2) { lom.gamepad2.rumble(1.0, 1.0, 500); lom.gamepad1.rumble(1.0, 1.0, 800); checkVibr = false }
            } else {
                vibeTime.reset()
                if (ro > 0) { lom.gamepad2.rumble(0.3, 0.3, 100); lom.gamepad1.rumble(0.3, 0.3, 100) }
            }
        }
        if (__LOG_STATUS) { logs("IntakeCurrent", intake.current) }
    }

    var status = 0
        set(v) {
            if (USE_INTAKE) {
                when (v) {
                    SNothing -> { sint(IntakeGet, 1, 0.0); checkVibr = false; __UPDATE_SENSORS = false }
                    SDown -> sint(IntakeGetUp, 0, 0.0)
                    SUp -> sint(IntakeGetUp, 1, 0.0)
                    SIntake -> { sint(IntakeGet, 0, IntakePower); checkVibr = true; __UPDATE_SENSORS = true }
                    SStack1 -> sint(IntakeStack12, 0)
                    SStack2 -> sint(IntakeStack12, 1)
                    SStack3 -> sint(IntakeStack34, 0)
                    SStack4 -> sint(IntakeStack34, 1)
                    SStack5 -> sint(IntakeStack56, 0)
                    SStack6 -> sint(IntakeStack56, 1, 0.0)
                    SInvert -> sint(IntakeGetCostac, 1, IntakeRevPower)
                    SIdleIntake -> sint(IntakeGetUp, 0)
                    SUpulLuiCostacu -> { sint(IntakeGetCostac, 0, 0.0); checkVibr = false; __UPDATE_SENSORS = false }
                    SUpulLuiCostacuIn -> sint(IntakeGetCostac, 0, -0.5)
                    SKeep -> sint(IntakeGet, 0, -0.3)
                    SInvert2 -> sint(IntakeGetCostac, 0, IntakeRevPower)
                    SFinalHang -> sint(IntakeHANGHANG, 0, 0.0)
                    else -> {}
                }
                field = v
            }
        }

    init { status = SNothing }
}