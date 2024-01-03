package org.firstinspires.ftc.teamcode

import com.acmerobotics.dashboard.config.Config
import com.acmerobotics.dashboard.telemetry.TelemetryPacket
import com.qualcomm.robotcore.eventloop.opmode.OpMode
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import com.qualcomm.robotcore.util.ElapsedTime
import org.firstinspires.ftc.teamcode.IHRP.AD
import org.firstinspires.ftc.teamcode.IHRP.AF
import org.firstinspires.ftc.teamcode.IHRP.AP
import org.firstinspires.ftc.teamcode.IHRP.KILL
import org.firstinspires.ftc.teamcode.IHRP.OFFSET
import org.firstinspires.ftc.teamcode.utils.RobotFuncs.clown
import org.firstinspires.ftc.teamcode.utils.RobotFuncs.controller
import org.firstinspires.ftc.teamcode.utils.RobotFuncs.dashboard
import org.firstinspires.ftc.teamcode.utils.RobotFuncs.diffy
import org.firstinspires.ftc.teamcode.utils.RobotFuncs.endma
import org.firstinspires.ftc.teamcode.utils.RobotFuncs.funkyL
import org.firstinspires.ftc.teamcode.utils.RobotFuncs.funkyR
import org.firstinspires.ftc.teamcode.utils.RobotFuncs.initma
import org.firstinspires.ftc.teamcode.utils.RobotFuncs.intake
import org.firstinspires.ftc.teamcode.utils.RobotFuncs.log_state
import org.firstinspires.ftc.teamcode.utils.RobotFuncs.preinit
import org.firstinspires.ftc.teamcode.utils.RobotFuncs.ridIntake
import org.firstinspires.ftc.teamcode.utils.RobotFuncs.send_log
import org.firstinspires.ftc.teamcode.utils.RobotFuncs.startma
import org.firstinspires.ftc.teamcode.utils.RobotFuncs.swerve
import org.firstinspires.ftc.teamcode.utils.RobotFuncs.timmy
import org.firstinspires.ftc.teamcode.utils.RobotVars.DIFDOWN
import org.firstinspires.ftc.teamcode.utils.RobotVars.DIFFDOWN
import org.firstinspires.ftc.teamcode.utils.RobotVars.DIFFUP
import org.firstinspires.ftc.teamcode.utils.RobotVars.DIFUP
import org.firstinspires.ftc.teamcode.utils.RobotVars.FUNKYLD
import org.firstinspires.ftc.teamcode.utils.RobotVars.FUNKYLU
import org.firstinspires.ftc.teamcode.utils.RobotVars.FUNKYRD
import org.firstinspires.ftc.teamcode.utils.RobotVars.FUNKYRU
import org.firstinspires.ftc.teamcode.utils.RobotVars.IntakePDown
import org.firstinspires.ftc.teamcode.utils.RobotVars.IntakePUp
import org.firstinspires.ftc.teamcode.utils.RobotVars.GhearaSDESCHIS
import org.firstinspires.ftc.teamcode.utils.RobotVars.GearaSINCHIS
import org.firstinspires.ftc.teamcode.utils.RobotVars.IntakePower
import org.firstinspires.ftc.teamcode.utils.Util.angDiff
import org.firstinspires.ftc.teamcode.utils.Util.angNorm
import org.firstinspires.ftc.teamcode.utils.Util.epsEq
import kotlin.math.abs
import kotlin.math.atan2
import kotlin.math.hypot

@Config
object IHRP {
    @JvmField
    var AP = 1.0

    @JvmField
    var AI = 0.0

    @JvmField
    var AD = 0.0

    @JvmField
    var AF = 0.0

    @JvmField
    var cswerve = false

    @JvmField
    var ccswerve = false

    @JvmField
    var ALF = 0.0

    @JvmField
    var ALB = 0.0

    @JvmField
    var ARF = 0.0

    @JvmField
    var ARB = 0.0

    @JvmField
    var SLF = 0.0

    @JvmField
    var SLB = 0.0

    @JvmField
    var SRF = 0.0

    @JvmField
    var SRB = 0.0

    @JvmField
    var KILL = 1.0

    @JvmField
    var OFFSET = 0.0
}

@TeleOp(name = "我討厭修訂")
class OpIHATEREV : OpMode() {

    override fun init() {
        preinit()
        initma(this)
    }

    override fun start() {
        startma()
        at.reset()
    }

    var ale = 0.0
    var at = ElapsedTime() // RB - RF
    var targetAngle = 0.0
    fun get_angf(): Double {
        return gamepad1.right_stick_x.toDouble() * KILL
        if (hypot(gamepad1.right_stick_y, gamepad1.right_stick_x) > 0.5) {
            targetAngle = atan2(gamepad1.right_stick_y, gamepad1.right_stick_x) + Math.PI / 2
        }
        var ae = angDiff(targetAngle, timmy.yaw)
        ae = if (abs(ae) < 0.1) 0.0 else ae
        val ad = (ae - ale) / at.seconds()
        at.reset()
        val cf = AP * ae + AD * ad
        val tp = TelemetryPacket()
        tp.put("Target", targetAngle)
        tp.put("TTimmy", timmy.yaw)
        tp.put("TError", ae)
        tp.put("TDeri", ad)
        tp.put("TFors", if (abs(cf) > 0.03) cf + (if (cf > 0.0) AF else -AF) else 0.0)
        dashboard.sendTelemetryPacket(tp)
        return if (abs(cf) > 0.03) cf + (if (cf > 0.0) AF else -AF) else 0.0
    }

    override fun loop() {
        controller.update()
        if (controller.C1A == controller.JUST_PRESSED) {
            swerve.locked = !swerve.locked
            swerve.move(0.1, 0.0, 0.0)
        }
        if (controller.C1B == controller.JUST_PRESSED) {
            swerve.maintainHeading = !swerve.maintainHeading
        }

        val speed = hypot(gamepad1.left_stick_x, gamepad1.left_stick_y).toDouble()
        val angle = angNorm(-atan2(gamepad1.left_stick_y, gamepad1.left_stick_x) + Math.PI / 2 + OFFSET - timmy.yaw)
        val correctAngForce = get_angf()
        val fcoef = 1.0 - gamepad1.right_trigger * 0.6
        swerve.move(speed * fcoef, angle, correctAngForce * fcoef)

        if (controller.C2RB == controller.JUST_PRESSED) {
            if (epsEq(clown.position, GhearaSDESCHIS)) {
                clown.position = GearaSINCHIS
            } else {
                clown.position = GhearaSDESCHIS
            }
        }

        if (controller.C2Y == controller.JUST_PRESSED) {
            if (epsEq(ridIntake.position, IntakePUp)) {
                intake.power = IntakePower
                ridIntake.position = IntakePDown
            } else {
                intake.power = 0.0
                ridIntake.position = IntakePUp
            }
        }

        if (controller.C2DU == controller.JUST_PRESSED) {
            if (epsEq(funkyL.position, FUNKYLD)) {
                funkyL.position = FUNKYLU
                funkyR.position = FUNKYRU
            } else {
                funkyL.position = FUNKYLD
                funkyR.position = FUNKYRD
            }
        }

        if (controller.C2A == controller.JUST_PRESSED) {
            diffy.targetPos = DIFUP
            diffy.targetDiff = DIFFUP
        }

        if (controller.C2X == controller.JUST_PRESSED) {
            diffy.targetPos = DIFUP
            diffy.targetDiff = DIFFDOWN
        }

        if (controller.C2B == controller.JUST_PRESSED) {
            diffy.targetPos = DIFDOWN
            diffy.targetDiff = DIFFUP
        }

        log_state()
        send_log()
    }

    override fun stop() = endma()

    /*
        LF = LF
        LB = RB
        RF = RF
        RB = LB
     */
}
