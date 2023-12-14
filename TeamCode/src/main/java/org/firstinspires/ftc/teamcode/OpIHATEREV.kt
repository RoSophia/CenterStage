package org.firstinspires.ftc.teamcode

import com.acmerobotics.dashboard.config.Config
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import com.qualcomm.robotcore.eventloop.opmode.OpMode
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import com.qualcomm.robotcore.util.ElapsedTime
import org.firstinspires.ftc.teamcode.IHRP.AD
import org.firstinspires.ftc.teamcode.IHRP.AF
import org.firstinspires.ftc.teamcode.IHRP.AI
import org.firstinspires.ftc.teamcode.IHRP.ALB
import org.firstinspires.ftc.teamcode.IHRP.ALF
import org.firstinspires.ftc.teamcode.IHRP.AP
import org.firstinspires.ftc.teamcode.IHRP.ARB
import org.firstinspires.ftc.teamcode.IHRP.ARF
import org.firstinspires.ftc.teamcode.IHRP.SLB
import org.firstinspires.ftc.teamcode.IHRP.SLF
import org.firstinspires.ftc.teamcode.IHRP.SRB
import org.firstinspires.ftc.teamcode.IHRP.SRF
import org.firstinspires.ftc.teamcode.IHRP.ccswerve
import org.firstinspires.ftc.teamcode.IHRP.cswerve
import org.firstinspires.ftc.teamcode.utils.RobotFuncs.controller
import org.firstinspires.ftc.teamcode.utils.RobotFuncs.endma
import org.firstinspires.ftc.teamcode.utils.RobotFuncs.initma
import org.firstinspires.ftc.teamcode.utils.RobotFuncs.log_state
import org.firstinspires.ftc.teamcode.utils.RobotFuncs.preinit
import org.firstinspires.ftc.teamcode.utils.RobotFuncs.startma
import org.firstinspires.ftc.teamcode.utils.RobotFuncs.swerve
import org.firstinspires.ftc.teamcode.utils.RobotFuncs.timmy
import org.firstinspires.ftc.teamcode.utils.Util.angDiff
import org.firstinspires.ftc.teamcode.utils.Util.angNorm
import org.firstinspires.ftc.teamcode.utils.Util.epsEq
import kotlin.math.abs
import kotlin.math.atan2
import kotlin.math.hypot

@Config
object IHRP {
    @JvmField
    var AP = 0.0

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
    fun get_angf(): Double {
        val targetAngle = atan2(gamepad1.left_stick_y, gamepad1.left_stick_x) - Math.PI / 2
        var ae = angDiff(targetAngle, timmy.yaw)
        ae = if (abs(ae) < 0.1) 0.0 else ae
        val ad = (ae - ale) / at.seconds()
        at.reset()
        val cf = AP * ae + AD * ad
        return if (abs(cf) > 0.03) cf + (if (cf > 0.0) AF else -AF) else 0.0
    }

    override fun loop() {
        controller.update()
        if (controller.C1A == 2) {
            swerve.locked = !swerve.locked
            swerve.move(0.1, 0.0, 0.0)
        }
        if (controller.C1B == 2) {
            swerve.maintainHeading = !swerve.maintainHeading
        }

        if (cswerve) {
            if (!epsEq(swerve.speed, 0.0)) {
                swerve.move(0.0, 0.0, 0.0)
            }
            swerve.lf.angle = ALF
            swerve.lb.angle = ALB
            swerve.rf.angle = ARF
            swerve.rb.angle = ARB
            swerve.lf.speed = SLF
            swerve.lb.speed = SLB
            swerve.rf.speed = SRF
            swerve.rb.speed = SRB
        } else if (ccswerve) {
            swerve.move(SLF, ALF, ALB)
        } else {
            val speed = hypot(gamepad1.left_stick_x, gamepad1.left_stick_y).toDouble()
            val angle = angNorm(-atan2(gamepad1.left_stick_y, gamepad1.left_stick_x) + Math.PI / 2 + timmy.yaw)
            val correctAngForce = get_angf()

            swerve.move(speed, angle, correctAngForce);
        }
        log_state()
    }

    override fun stop() = endma()

    /*
        LF = LF
        LB = RB
        RF = RF
        RB = LB
     */
}
