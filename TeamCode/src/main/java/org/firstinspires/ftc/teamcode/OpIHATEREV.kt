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
import org.firstinspires.ftc.teamcode.utils.Util.epsEq
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

    var ale = 0.0
    var ai = 0.0
    var at = ElapsedTime() // RB - RF

    override fun start() {
        startma()
        at.reset()
    }

    override fun loop() {
        controller.update()
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
        } else {
            val speed = hypot(gamepad1.left_stick_x, gamepad1.left_stick_y).toDouble()
            val angle = atan2(gamepad1.left_stick_y, gamepad1.left_stick_x) - Math.PI / 4 + timmy.yaw
            val targetAngle = atan2(gamepad2.left_stick_y, gamepad2.left_stick_x) - Math.PI / 4

            val ae = angDiff(targetAngle, timmy.yaw)
            val ad = (ae - ale) / at.seconds()
            ale = ae
            ai += ae * at.seconds()

            swerve.move(speed, angle, AP * ae + ad * AD + ai * AI + AF)
            at.reset()
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
